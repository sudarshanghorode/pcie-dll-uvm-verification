`timescale 1ns/1ps

//------------------------------------------------------------------------------
// Helpers shared by the transmitter and the receiver
//------------------------------------------------------------------------------
package pcie_dll_pkg;

  localparam logic [31:0] CRC_POLY = 32'h04C11DB7;

  // One 32-bit step of an MSB-first CRC-32 (polynomial 0x04C11DB7)
  function automatic logic [31:0] crc32_step(input logic [31:0] crc_in,
                                             input logic [31:0] dw);
    logic [31:0] c;
    c = crc_in ^ dw;
    for (int i = 0; i < 32; i++) begin
      if (c[31]) c = (c << 1) ^ CRC_POLY;
      else       c = (c << 1);
    end
    return c;
  endfunction

  // LCRC over {sequence number, TLP double-words}
  function automatic logic [31:0] calc_lcrc(input logic [11:0]  seq,
                                            input logic [2:0]   len,
                                            input logic [127:0] data);
    logic [31:0] c;
    c = 32'hFFFF_FFFF;
    c = crc32_step(c, {20'h0, seq});
    for (int i = 0; i < 4; i++) begin
      if (i < int'(len)) c = crc32_step(c, data[32*i +: 32]);
    end
    return ~c;
  endfunction

endpackage

//------------------------------------------------------------------------------
// DLL transmitter
//   - assigns a 12-bit sequence number to every TLP and appends a 32-bit LCRC
//   - keeps every unacknowledged TLP in a replay buffer
//   - ACK  : frees all TLPs up to and including the acknowledged sequence number
//   - NAK  : frees acknowledged TLPs, then replays the rest, oldest first
//   - replay timer: replays everything unacknowledged if nothing is acknowledged
//                   for TIMEOUT clocks (recovers from a lost TLP or lost DLLP)
//------------------------------------------------------------------------------
module pcie_dll_tx #(
  parameter int DEPTH   = 8,     // replay buffer entries (power of two)
  parameter int TIMEOUT = 64     // replay timer, in clocks
)(
  input  logic         clk,
  input  logic         rst_n,
  // transaction-layer side
  input  logic         tlp_valid,
  output logic         tlp_ready,
  input  logic [127:0] tlp_data,       // up to 4 double-words
  input  logic [2:0]   tlp_len,        // number of valid double-words, 1..4
  // link side (one framed TLP per clock)
  output logic         lnk_valid,
  output logic [11:0]  lnk_seq,
  output logic [2:0]   lnk_len,
  output logic [127:0] lnk_data,
  output logic [31:0]  lnk_lcrc,
  // returned DLLPs
  input  logic         dllp_valid,
  input  logic         dllp_is_nak,
  input  logic [11:0]  dllp_seq,
  // observation
  output logic [3:0]   occupancy,      // unacknowledged TLPs in the replay buffer
  output logic [1:0]   replay_ev       // pulse: [0] replay caused by NAK, [1] by timeout
);
  import pcie_dll_pkg::*;

  localparam int PW = $clog2(DEPTH);

  logic [127:0]  rb_data [DEPTH];
  logic [2:0]    rb_len  [DEPTH];
  logic [11:0]   rb_seq  [DEPTH];
  logic [31:0]   rb_lcrc [DEPTH];

  logic [PW-1:0] wr_ptr;       // next free entry
  logic [PW-1:0] ack_ptr;      // oldest unacknowledged entry
  logic [PW-1:0] tx_ptr;       // next entry to put on the link
  logic [PW:0]   count;        // entries held (unacknowledged)
  logic [PW:0]   unsent;       // entries still to be sent (or re-sent)
  logic [11:0]   next_seq;
  int unsigned   timer;

  assign tlp_ready = rst_n && (count < DEPTH);
  assign occupancy = count;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      wr_ptr    <= '0;
      ack_ptr   <= '0;
      tx_ptr    <= '0;
      count     <= '0;
      unsent    <= '0;
      next_seq  <= '0;
      timer     <= 0;
      lnk_valid <= 1'b0;
      lnk_seq   <= '0;
      lnk_len   <= '0;
      lnk_data  <= '0;
      lnk_lcrc  <= '0;
      replay_ev <= 2'b00;
    end else begin : tx_main
      logic [PW:0]   n_count;
      logic [PW:0]   n_unsent;
      logic [PW:0]   sent;
      logic [PW:0]   nfree;
      logic [PW-1:0] n_wr;
      logic [PW-1:0] n_ack;
      logic [PW-1:0] n_tx;
      logic [11:0]   n_seq;
      logic [11:0]   oldest;
      logic [11:0]   diff;
      logic          progress;
      int unsigned   n_timer;

      n_count  = count;
      n_unsent = unsent;
      n_wr     = wr_ptr;
      n_ack    = ack_ptr;
      n_tx     = tx_ptr;
      n_seq    = next_seq;
      n_timer  = timer;
      progress = 1'b0;

      lnk_valid <= 1'b0;
      replay_ev <= 2'b00;

      // (1) returned ACK / NAK
      if (dllp_valid && count != 0) begin
        oldest = next_seq - count;
        diff   = dllp_seq - oldest;
        if (diff < count) begin                  // acknowledges 1..count entries
          nfree   = diff + 1'b1;
          sent    = count - unsent;
          n_ack   = ack_ptr + nfree[PW-1:0];
          n_count = count - nfree;
          if (nfree > sent) begin                // acknowledged more than was re-sent so far
            n_tx     = n_ack;
            n_unsent = n_count;
          end
          progress = 1'b1;
        end
        if (dllp_is_nak && n_count != 0) begin   // NAK: replay everything unacknowledged
          n_tx      = n_ack;
          n_unsent  = n_count;
          replay_ev <= 2'b01;
          progress  = 1'b1;
        end
      end

      // (2) replay timer
      if (n_count == 0 || progress) n_timer = 0;
      else                          n_timer = n_timer + 1;
      if (n_count != 0 && n_timer >= TIMEOUT) begin
        n_tx      = n_ack;
        n_unsent  = n_count;
        n_timer   = 0;
        replay_ev <= 2'b10;
      end

      // (3) put the next (or replayed) TLP on the link
      if (n_unsent != 0) begin
        lnk_valid <= 1'b1;
        lnk_seq   <= rb_seq[n_tx];
        lnk_len   <= rb_len[n_tx];
        lnk_data  <= rb_data[n_tx];
        lnk_lcrc  <= rb_lcrc[n_tx];
        n_tx      = n_tx + 1'b1;
        n_unsent  = n_unsent - 1'b1;
      end

      // (4) accept a new TLP from the transaction layer
      if (tlp_valid && tlp_ready) begin
        rb_data[wr_ptr] <= tlp_data;
        rb_len[wr_ptr]  <= tlp_len;
        rb_seq[wr_ptr]  <= next_seq;
        rb_lcrc[wr_ptr] <= calc_lcrc(next_seq, tlp_len, tlp_data);
        n_wr     = wr_ptr + 1'b1;
        n_seq    = next_seq + 1'b1;
        n_count  = n_count + 1'b1;
        n_unsent = n_unsent + 1'b1;
      end

      count    <= n_count;
      unsent   <= n_unsent;
      wr_ptr   <= n_wr;
      ack_ptr  <= n_ack;
      tx_ptr   <= n_tx;
      next_seq <= n_seq;
      timer    <= n_timer;
    end
  end

endmodule

//------------------------------------------------------------------------------
// DLL receiver
//   - checks the LCRC and the sequence number of every received TLP
//   - good TLP in order : delivered upward, ACK(seq) returned
//   - duplicate         : discarded, ACK(last good seq) returned
//   - bad LCRC or gap   : discarded, NAK(last good seq) returned (one NAK until a
//                         good TLP is received again, as in the PCIe NAK_SCHEDULED rule)
//------------------------------------------------------------------------------
module pcie_dll_rx (
  input  logic         clk,
  input  logic         rst_n,
  // link side
  input  logic         lnk_valid,
  input  logic [11:0]  lnk_seq,
  input  logic [2:0]   lnk_len,
  input  logic [127:0] lnk_data,
  input  logic [31:0]  lnk_lcrc,
  // transaction-layer side
  output logic         tlp_valid,
  output logic [127:0] tlp_data,
  output logic [2:0]   tlp_len,
  // DLLPs to send back
  output logic         dllp_valid,
  output logic         dllp_is_nak,
  output logic [11:0]  dllp_seq
);
  import pcie_dll_pkg::*;

  logic [11:0] expected_seq;
  logic        nak_sched;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      expected_seq <= '0;
      nak_sched    <= 1'b0;
      tlp_valid    <= 1'b0;
      tlp_data     <= '0;
      tlp_len      <= '0;
      dllp_valid   <= 1'b0;
      dllp_is_nak  <= 1'b0;
      dllp_seq     <= '0;
    end else begin : rx_main
      logic        lcrc_ok;
      logic [11:0] seq_dist;

      tlp_valid  <= 1'b0;
      dllp_valid <= 1'b0;

      if (lnk_valid) begin
        lcrc_ok = (calc_lcrc(lnk_seq, lnk_len, lnk_data) == lnk_lcrc);
        seq_dist    = expected_seq - lnk_seq;

        if (lcrc_ok && seq_dist == 12'd0) begin
          // good TLP, in order
          tlp_valid    <= 1'b1;
          tlp_data     <= lnk_data;
          tlp_len      <= lnk_len;
          expected_seq <= expected_seq + 12'd1;
          nak_sched    <= 1'b0;
          dllp_valid   <= 1'b1;
          dllp_is_nak  <= 1'b0;
          dllp_seq     <= lnk_seq;
        end else if (lcrc_ok && seq_dist <= 12'd2048) begin
          // duplicate of a TLP that was already received: drop it, re-ACK
          dllp_valid   <= 1'b1;
          dllp_is_nak  <= 1'b0;
          dllp_seq     <= expected_seq - 12'd1;
        end else if (!nak_sched) begin
          // bad LCRC, or a sequence-number gap: ask for a replay
          dllp_valid   <= 1'b1;
          dllp_is_nak  <= 1'b1;
          dllp_seq     <= expected_seq - 12'd1;
          nak_sched    <= 1'b1;
        end
      end
    end
  end

endmodule

//------------------------------------------------------------------------------
// Link between transmitter and receiver, with error injection
//   inj_mode sampled with every transmitted TLP:
//     0 = clean, 1 = flip a bit of the LCRC, 2 = drop the TLP, 3 = flip a data bit
//------------------------------------------------------------------------------
module pcie_dll_channel (
  input  logic         clk,
  input  logic         rst_n,
  input  logic [1:0]   inj_mode,
  input  logic         lnk_valid_i,
  input  logic [11:0]  lnk_seq_i,
  input  logic [2:0]   lnk_len_i,
  input  logic [127:0] lnk_data_i,
  input  logic [31:0]  lnk_lcrc_i,
  output logic         lnk_valid_o,
  output logic [11:0]  lnk_seq_o,
  output logic [2:0]   lnk_len_o,
  output logic [127:0] lnk_data_o,
  output logic [31:0]  lnk_lcrc_o,
  output logic [1:0]   ev_kind          // what happened to the TLP now on the input
);

  assign ev_kind = lnk_valid_i ? inj_mode : 2'd0;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      lnk_valid_o <= 1'b0;
      lnk_seq_o   <= '0;
      lnk_len_o   <= '0;
      lnk_data_o  <= '0;
      lnk_lcrc_o  <= '0;
    end else begin
      lnk_valid_o <= lnk_valid_i && (inj_mode != 2'd2);
      lnk_seq_o   <= lnk_seq_i;
      lnk_len_o   <= lnk_len_i;
      lnk_data_o  <= (inj_mode == 2'd3) ? (lnk_data_i ^ 128'h1) : lnk_data_i;
      lnk_lcrc_o  <= (inj_mode == 2'd1) ? (lnk_lcrc_i ^ 32'h1)  : lnk_lcrc_i;
    end
  end

endmodule

//------------------------------------------------------------------------------
// Device under test: transmitter -> channel -> receiver, plus the DLLP return path
//------------------------------------------------------------------------------
module pcie_dll_top (
  input  logic         clk,
  input  logic         rst_n,
  // transaction-layer side of the transmitter
  input  logic         in_valid,
  output logic         in_ready,
  input  logic [127:0] in_data,
  input  logic [2:0]   in_len,
  // transaction-layer side of the receiver
  output logic         out_valid,
  output logic [127:0] out_data,
  output logic [2:0]   out_len,
  // link impairment control
  input  logic [1:0]   inj_mode,
  // observation points used by the testbench
  output logic         dbg_lnk_valid,
  output logic [11:0]  dbg_lnk_seq,
  output logic [2:0]   dbg_lnk_len,
  output logic [1:0]   dbg_beat_err,
  output logic         dbg_dllp_valid,
  output logic         dbg_dllp_is_nak,
  output logic [11:0]  dbg_dllp_seq,
  output logic [3:0]   dbg_count,
  output logic [1:0]   dbg_replay_ev
);

  // transmitter -> channel
  logic         t_lnk_valid;
  logic [11:0]  t_lnk_seq;
  logic [2:0]   t_lnk_len;
  logic [127:0] t_lnk_data;
  logic [31:0]  t_lnk_lcrc;
  // channel -> receiver
  logic         c_lnk_valid;
  logic [11:0]  c_lnk_seq;
  logic [2:0]   c_lnk_len;
  logic [127:0] c_lnk_data;
  logic [31:0]  c_lnk_lcrc;
  // receiver -> DLLP return path -> transmitter
  logic         r_dllp_valid;
  logic         r_dllp_is_nak;
  logic [11:0]  r_dllp_seq;
  logic [13:0]  dllp_p0, dllp_p1;
  logic         d_dllp_valid;
  logic         d_dllp_is_nak;
  logic [11:0]  d_dllp_seq;

  pcie_dll_tx #(.DEPTH(8), .TIMEOUT(64)) u_tx (
    .clk(clk), .rst_n(rst_n),
    .tlp_valid(in_valid), .tlp_ready(in_ready), .tlp_data(in_data), .tlp_len(in_len),
    .lnk_valid(t_lnk_valid), .lnk_seq(t_lnk_seq), .lnk_len(t_lnk_len),
    .lnk_data(t_lnk_data), .lnk_lcrc(t_lnk_lcrc),
    .dllp_valid(d_dllp_valid), .dllp_is_nak(d_dllp_is_nak), .dllp_seq(d_dllp_seq),
    .occupancy(dbg_count), .replay_ev(dbg_replay_ev)
  );

  pcie_dll_channel u_ch (
    .clk(clk), .rst_n(rst_n), .inj_mode(inj_mode),
    .lnk_valid_i(t_lnk_valid), .lnk_seq_i(t_lnk_seq), .lnk_len_i(t_lnk_len),
    .lnk_data_i(t_lnk_data), .lnk_lcrc_i(t_lnk_lcrc),
    .lnk_valid_o(c_lnk_valid), .lnk_seq_o(c_lnk_seq), .lnk_len_o(c_lnk_len),
    .lnk_data_o(c_lnk_data), .lnk_lcrc_o(c_lnk_lcrc),
    .ev_kind(dbg_beat_err)
  );

  pcie_dll_rx u_rx (
    .clk(clk), .rst_n(rst_n),
    .lnk_valid(c_lnk_valid), .lnk_seq(c_lnk_seq), .lnk_len(c_lnk_len),
    .lnk_data(c_lnk_data), .lnk_lcrc(c_lnk_lcrc),
    .tlp_valid(out_valid), .tlp_data(out_data), .tlp_len(out_len),
    .dllp_valid(r_dllp_valid), .dllp_is_nak(r_dllp_is_nak), .dllp_seq(r_dllp_seq)
  );

  // two register stages model the latency of the return path
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      dllp_p0 <= '0;
      dllp_p1 <= '0;
    end else begin
      dllp_p0 <= {r_dllp_valid, r_dllp_is_nak, r_dllp_seq};
      dllp_p1 <= dllp_p0;
    end
  end

  assign d_dllp_valid  = dllp_p1[13];
  assign d_dllp_is_nak = dllp_p1[12];
  assign d_dllp_seq    = dllp_p1[11:0];

  assign dbg_lnk_valid   = t_lnk_valid;
  assign dbg_lnk_seq     = t_lnk_seq;
  assign dbg_lnk_len     = t_lnk_len;
  assign dbg_dllp_valid  = d_dllp_valid;
  assign dbg_dllp_is_nak = d_dllp_is_nak;
  assign dbg_dllp_seq    = d_dllp_seq;

endmodule

//------------------------------------------------------------------------------
// Interface: signals, clocking blocks for the UVM agent, assertions and covers
//------------------------------------------------------------------------------
interface pcie_dll_if (input logic ACLK, input logic ARESETn);

  // driven by the testbench
  logic         in_valid = 1'b0;
  logic [127:0] in_data  = '0;
  logic [2:0]   in_len   = 3'd1;
  logic [1:0]   inj_mode = 2'd0;

  // driven by the DUT
  logic         in_ready;
  logic         out_valid;
  logic [127:0] out_data;
  logic [2:0]   out_len;
  logic         dbg_lnk_valid;
  logic [11:0]  dbg_lnk_seq;
  logic [2:0]   dbg_lnk_len;
  logic [1:0]   dbg_beat_err;
  logic         dbg_dllp_valid;
  logic         dbg_dllp_is_nak;
  logic [11:0]  dbg_dllp_seq;
  logic [3:0]   dbg_count;
  logic [1:0]   dbg_replay_ev;

  // TLP driver
  clocking drv_cb @(posedge ACLK);
    default input #1step output #1ns;
    output in_valid, in_data, in_len;
    input  in_ready;
  endclocking

  // link error injector
  clocking inj_cb @(posedge ACLK);
    default input #1step output #1ns;
    output inj_mode;
    input  dbg_lnk_valid, dbg_beat_err;
  endclocking

  // monitor: everything is an input
  clocking mon_cb @(posedge ACLK);
    default input #1step;
    input in_valid, in_ready, in_data, in_len,
          out_valid, out_data, out_len,
          dbg_lnk_valid, dbg_lnk_seq, dbg_lnk_len, dbg_beat_err,
          dbg_dllp_valid, dbg_dllp_is_nak, dbg_dllp_seq,
          dbg_count, dbg_replay_ev;
  endclocking

  //---------------------------- SystemVerilog Assertions ----------------------
  // a TLP offered to the transmitter must be held, unchanged, until it is accepted
  a_in_hold : assert property (@(posedge ACLK) disable iff (!ARESETn)
      (in_valid && !in_ready) |=> (in_valid && $stable(in_data) && $stable(in_len)))
      else $error("PCIe-DLL: TLP changed before it was accepted");

  // only lengths 1..4 double-words exist
  a_in_len_legal : assert property (@(posedge ACLK) disable iff (!ARESETn)
      in_valid |-> (in_len >= 3'd1 && in_len <= 3'd4))
      else $error("PCIe-DLL: illegal TLP length offered");

  a_out_len_legal : assert property (@(posedge ACLK) disable iff (!ARESETn)
      out_valid |-> (out_len >= 3'd1 && out_len <= 3'd4))
      else $error("PCIe-DLL: illegal TLP length delivered");

  // the replay buffer must never overflow, and a full buffer must stall the sender
  a_no_overflow : assert property (@(posedge ACLK) disable iff (!ARESETn)
      dbg_count <= 4'd8)
      else $error("PCIe-DLL: replay buffer overflow");

  a_full_stalls : assert property (@(posedge ACLK) disable iff (!ARESETn)
      (dbg_count == 4'd8) |-> !in_ready)
      else $error("PCIe-DLL: sender not stalled with a full replay buffer");

  // a replay only makes sense when something is waiting for acknowledgement
  a_replay_has_data : assert property (@(posedge ACLK) disable iff (!ARESETn)
      (dbg_replay_ev != 2'b00) |-> (dbg_count != 4'd0))
      else $error("PCIe-DLL: replay with an empty replay buffer");

  // handshake and DLLP fields must never be unknown after reset
  a_no_x_ctrl : assert property (@(posedge ACLK) disable iff (!ARESETn)
      !$isunknown({in_ready, out_valid, dbg_lnk_valid, dbg_dllp_valid}))
      else $error("PCIe-DLL: unknown value on a control signal");

  a_dllp_seq_known : assert property (@(posedge ACLK) disable iff (!ARESETn)
      dbg_dllp_valid |-> !$isunknown({dbg_dllp_is_nak, dbg_dllp_seq}))
      else $error("PCIe-DLL: unknown DLLP fields");

  //---------------------------- cover properties ------------------------------
  c_nak_returned : cover property (@(posedge ACLK) disable iff (!ARESETn)
      dbg_dllp_valid && dbg_dllp_is_nak);
  c_replay_by_nak : cover property (@(posedge ACLK) disable iff (!ARESETn)
      dbg_replay_ev[0]);
  c_replay_by_timeout : cover property (@(posedge ACLK) disable iff (!ARESETn)
      dbg_replay_ev[1]);
  c_buffer_full : cover property (@(posedge ACLK) disable iff (!ARESETn)
      dbg_count == 4'd8);
  c_sender_stalled : cover property (@(posedge ACLK) disable iff (!ARESETn)
      in_valid && !in_ready);
  c_seq_wraparound : cover property (@(posedge ACLK) disable iff (!ARESETn)
      (dbg_lnk_valid && dbg_lnk_seq == 12'hFFF) ##[1:$] (dbg_lnk_valid && dbg_lnk_seq == 12'h000));

endinterface
