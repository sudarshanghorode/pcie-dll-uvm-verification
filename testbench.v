`timescale 1ns/1ps
`include "uvm_macros.svh"

package pcie_dll_tb_pkg;
  import uvm_pkg::*;

  // two analysis imps in one component (TLPs accepted / TLPs delivered)
  `uvm_analysis_imp_decl(_in)
  `uvm_analysis_imp_decl(_out)

  //============================================================================
  // Sequence item: one TLP handed to the data link layer
  //============================================================================
  class pcie_tlp_item extends uvm_sequence_item;
    rand bit [127:0]  data;     // up to 4 double-words, unused double-words are zero
    rand bit [2:0]    len;      // number of valid double-words
    rand int unsigned gap;      // idle clocks before the TLP is offered

    // Pick the length first. Without this the solver weighs every (len, data) pair
    // equally, so len = 4 (2^128 data values) wins almost every time and TLPs of
    // 1..3 double-words are barely generated.
    constraint c_len  { solve len before data; len inside {[1:4]}; }
    constraint c_mask {
      (len < 2) -> data[127:32] == 0;
      (len < 3) -> data[127:64] == 0;
      (len < 4) -> data[127:96] == 0;
    }
    constraint c_gap  { gap dist { 0 := 70, [1:3] :/ 20, [4:12] :/ 10 }; }

    `uvm_object_utils(pcie_tlp_item)

    function new(string name = "pcie_tlp_item");
      super.new(name);
    endfunction

    function string convert2string();
      return $sformatf("TLP len=%0d data=0x%032h", len, data);
    endfunction
  endclass

  //============================================================================
  // Sequences
  //============================================================================
  // Directed: every length, corner payloads, and a burst larger than the buffer
  class pcie_dll_directed_seq extends uvm_sequence #(pcie_tlp_item);
    `uvm_object_utils(pcie_dll_directed_seq)

    function new(string name = "pcie_dll_directed_seq");
      super.new(name);
    endfunction

    function bit [127:0] mask_dw(int unsigned l);
      case (l)
        1:       return 128'h0000_0000_0000_0000_0000_0000_FFFF_FFFF;
        2:       return 128'h0000_0000_0000_0000_FFFF_FFFF_FFFF_FFFF;
        3:       return 128'h0000_0000_FFFF_FFFF_FFFF_FFFF_FFFF_FFFF;
        default: return 128'hFFFF_FFFF_FFFF_FFFF_FFFF_FFFF_FFFF_FFFF;
      endcase
    endfunction

    task do_tlp(int unsigned l, bit [127:0] d, int unsigned g);
      pcie_tlp_item t;
      t = pcie_tlp_item::type_id::create("t");
      start_item(t);
      if (!t.randomize() with { len == l; data == d; gap == g; }) begin
        `uvm_fatal("SEQ", "randomize failed")
      end
      finish_item(t);
    endtask

    task body();
      // one TLP of every legal length
      for (int l = 1; l <= 4; l++)
        do_tlp(l, 128'hDEAD_BEEF_CAFE_F00D_0123_4567_89AB_CDEF & mask_dw(l), 2);
      // corner payloads
      do_tlp(4, 128'h0, 1);
      do_tlp(4, 128'hFFFF_FFFF_FFFF_FFFF_FFFF_FFFF_FFFF_FFFF, 1);
      do_tlp(1, 128'h0, 1);
      do_tlp(1, 128'h0000_0000_0000_0000_0000_0000_FFFF_FFFF, 1);
      // back-to-back burst, longer than the replay buffer
      for (int i = 0; i < 12; i++)
        do_tlp(1 + (i % 4), 128'(i + 1) & mask_dw(1 + (i % 4)), 0);
    endtask
  endclass

  // Constrained-random traffic
  class pcie_dll_rand_seq extends uvm_sequence #(pcie_tlp_item);
    `uvm_object_utils(pcie_dll_rand_seq)
    int unsigned num_txn      = 100;
    bit          back_to_back = 1'b0;     // force gap = 0 to keep the sender saturated

    function new(string name = "pcie_dll_rand_seq");
      super.new(name);
    endfunction

    task body();
      pcie_tlp_item t;
      repeat (num_txn) begin
        t = pcie_tlp_item::type_id::create("t");
        start_item(t);
        if (back_to_back) begin
          if (!t.randomize() with { gap == 0; }) begin
            `uvm_fatal("SEQ", "randomize failed")
          end
        end else begin
          if (!t.randomize()) begin
            `uvm_fatal("SEQ", "randomize failed")
          end
        end
        finish_item(t);
      end
    endtask
  endclass

  //============================================================================
  // Driver: offers TLPs to the transmitter, honouring in_ready
  //============================================================================
  class pcie_dll_driver extends uvm_driver #(pcie_tlp_item);
    `uvm_component_utils(pcie_dll_driver)

    virtual pcie_dll_if vif;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      if (!uvm_config_db #(virtual pcie_dll_if)::get(this, "", "vif", vif)) begin
        `uvm_fatal("DRV", "virtual interface 'vif' not set")
      end
    endfunction

    task run_phase(uvm_phase phase);
      wait (vif.ARESETn === 1'b1);
      repeat (2) @(vif.drv_cb);
      forever begin
        seq_item_port.get_next_item(req);
        drive_tlp(req);
        seq_item_port.item_done();
      end
    endtask

    task drive_tlp(pcie_tlp_item t);
      repeat (t.gap) @(vif.drv_cb);
      vif.drv_cb.in_data  <= t.data;
      vif.drv_cb.in_len   <= t.len;
      vif.drv_cb.in_valid <= 1'b1;
      do begin @(vif.drv_cb); end while (!vif.drv_cb.in_ready);
      vif.drv_cb.in_valid <= 1'b0;
    endtask
  endclass

  //============================================================================
  // Link error injector: corrupts or drops TLPs on the link, at random
  //============================================================================
  class pcie_dll_err_inj extends uvm_component;
    `uvm_component_utils(pcie_dll_err_inj)

    virtual pcie_dll_if vif;
    int unsigned corrupt_pct = 0;      // % of link beats with a corrupted LCRC / data bit
    int unsigned drop_pct    = 0;      // % of link beats that are lost
    bit          drop_next   = 1'b0;   // drop exactly the next link beat, then clear

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      if (!uvm_config_db #(virtual pcie_dll_if)::get(this, "", "vif", vif)) begin
        `uvm_fatal("INJ", "virtual interface 'vif' not set")
      end
    endfunction

    task run_phase(uvm_phase phase);
      int unsigned r;
      wait (vif.ARESETn === 1'b1);
      forever begin
        @(vif.inj_cb);
        if (drop_next) begin
          if (vif.inj_cb.dbg_lnk_valid && vif.inj_cb.dbg_beat_err == 2'd2) begin
            drop_next = 1'b0;                       // the beat has just been dropped
            vif.inj_cb.inj_mode <= 2'd0;
          end else begin
            vif.inj_cb.inj_mode <= 2'd2;
          end
        end else begin
          r = $urandom_range(99, 0);
          if (r < corrupt_pct)
            vif.inj_cb.inj_mode <= ($urandom_range(1, 0) ? 2'd1 : 2'd3);
          else if (r < corrupt_pct + drop_pct)
            vif.inj_cb.inj_mode <= 2'd2;
          else
            vif.inj_cb.inj_mode <= 2'd0;
        end
      end
    endtask
  endclass

  //============================================================================
  // Monitor: TLPs accepted by the sender, TLPs delivered by the receiver,
  //          and link-level event counters
  //============================================================================
  class pcie_dll_monitor extends uvm_monitor;
    `uvm_component_utils(pcie_dll_monitor)

    virtual pcie_dll_if                vif;
    uvm_analysis_port #(pcie_tlp_item) ap_in;
    uvm_analysis_port #(pcie_tlp_item) ap_out;

    int unsigned n_beats, n_corrupt, n_drop, n_ack, n_nak, n_replay_nak, n_replay_to;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      ap_in  = new("ap_in", this);
      ap_out = new("ap_out", this);
      if (!uvm_config_db #(virtual pcie_dll_if)::get(this, "", "vif", vif)) begin
        `uvm_fatal("MON", "virtual interface 'vif' not set")
      end
    endfunction

    task run_phase(uvm_phase phase);
      pcie_tlp_item it;
      forever begin
        @(vif.mon_cb);

        if (vif.mon_cb.in_valid && vif.mon_cb.in_ready) begin
          it      = pcie_tlp_item::type_id::create("mon_in");
          it.data = vif.mon_cb.in_data;
          it.len  = vif.mon_cb.in_len;
          ap_in.write(it);
        end

        if (vif.mon_cb.out_valid) begin
          it      = pcie_tlp_item::type_id::create("mon_out");
          it.data = vif.mon_cb.out_data;
          it.len  = vif.mon_cb.out_len;
          ap_out.write(it);
        end

        if (vif.mon_cb.dbg_lnk_valid) begin
          n_beats++;
          case (vif.mon_cb.dbg_beat_err)
            2'd1, 2'd3: n_corrupt++;
            2'd2:       n_drop++;
            default: ;
          endcase
        end

        if (vif.mon_cb.dbg_dllp_valid) begin
          if (vif.mon_cb.dbg_dllp_is_nak) n_nak++;
          else                            n_ack++;
        end

        if (vif.mon_cb.dbg_replay_ev[0]) n_replay_nak++;
        if (vif.mon_cb.dbg_replay_ev[1]) n_replay_to++;
      end
    endtask

    function void report_phase(uvm_phase phase);
      `uvm_info("MON", $sformatf("LINK: %0d beats | injected: %0d corrupted, %0d dropped | DLLPs: %0d ACK, %0d NAK | replays: %0d by NAK, %0d by timeout",
                                 n_beats, n_corrupt, n_drop, n_ack, n_nak, n_replay_nak, n_replay_to), UVM_NONE)
      if ((n_corrupt + n_drop) > 0 && (n_replay_nak + n_replay_to) == 0) begin
        `uvm_error("MON", "errors were injected on the link but no replay ever happened")
      end
    endfunction
  endclass

  //============================================================================
  // Agent
  //============================================================================
  class pcie_dll_agent extends uvm_agent;
    `uvm_component_utils(pcie_dll_agent)

    pcie_dll_driver                drv;
    uvm_sequencer #(pcie_tlp_item) sqr;
    pcie_dll_monitor               mon;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      drv = pcie_dll_driver::type_id::create("drv", this);
      sqr = uvm_sequencer #(pcie_tlp_item)::type_id::create("sqr", this);
      mon = pcie_dll_monitor::type_id::create("mon", this);
    endfunction

    function void connect_phase(uvm_phase phase);
      super.connect_phase(phase);
      drv.seq_item_port.connect(sqr.seq_item_export);
    endfunction
  endclass

  //============================================================================
  // Scoreboard: every accepted TLP must be delivered exactly once, in order,
  //             with unchanged contents, whatever the link did to it
  //============================================================================
  class pcie_dll_scoreboard extends uvm_scoreboard;
    `uvm_component_utils(pcie_dll_scoreboard)

    uvm_analysis_imp_in  #(pcie_tlp_item, pcie_dll_scoreboard) in_export;
    uvm_analysis_imp_out #(pcie_tlp_item, pcie_dll_scoreboard) out_export;

    pcie_tlp_item exp_q[$];
    int unsigned  n_in, n_out, n_err;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      in_export  = new("in_export", this);
      out_export = new("out_export", this);
    endfunction

    function void write_in(pcie_tlp_item t);
      n_in++;
      exp_q.push_back(t);
    endfunction

    function void write_out(pcie_tlp_item t);
      pcie_tlp_item e;
      n_out++;
      if (exp_q.size() == 0) begin
        n_err++;
        `uvm_error("SCB", $sformatf("TLP delivered with none outstanding (duplicate?): %s", t.convert2string()))
        return;
      end
      e = exp_q.pop_front();
      if (e.len !== t.len || e.data !== t.data) begin
        n_err++;
        `uvm_error("SCB", $sformatf("TLP mismatch (lost, reordered or corrupted)\n  expected: %s\n  got     : %s",
                                    e.convert2string(), t.convert2string()))
      end
    endfunction

    function void check_phase(uvm_phase phase);
      if (exp_q.size() != 0) begin
        n_err++;
        `uvm_error("SCB", $sformatf("%0d TLPs were accepted but never delivered", exp_q.size()))
      end
    endfunction

    function void report_phase(uvm_phase phase);
      if (n_err == 0 && n_in > 0 && n_in == n_out) begin
        `uvm_info("SCB", $sformatf("PASS: %0d TLPs accepted, %0d delivered in order, exactly once, 0 mismatches", n_in, n_out), UVM_NONE)
      end else begin
        `uvm_error("SCB", $sformatf("FAIL: %0d errors, %0d TLPs accepted, %0d delivered", n_err, n_in, n_out))
      end
    endfunction
  endclass

  //============================================================================
  // Functional coverage
  //   cg_tlp  : sampled per accepted TLP
  //   cg_link : sampled every clock from the link and DLLP activity
  //============================================================================
  class pcie_dll_coverage extends uvm_subscriber #(pcie_tlp_item);
    `uvm_component_utils(pcie_dll_coverage)

    virtual pcie_dll_if vif;

    // values sampled every clock for cg_link
    bit          f_lnk_v;
    bit [2:0]    f_lnk_len;
    bit [1:0]    f_beat_err;
    bit [11:0]   f_seq;
    bit          f_dllp_v;
    bit          f_dllp_nak;
    bit [1:0]    f_rep;
    bit [3:0]    f_occ;
    bit          f_stall;

    covergroup cg_tlp with function sample(pcie_tlp_item t);
      option.per_instance = 1;
      option.name         = "pcie_dll_tlp_cov";

      cp_len : coverpoint t.len {
        bins len[] = {[1:4]};
      }
    endgroup

    covergroup cg_link;
      option.per_instance = 1;
      option.name         = "pcie_dll_link_cov";

      // what the link did to each transmitted TLP, per TLP length
      cp_beat_len : coverpoint f_lnk_len iff (f_lnk_v) {
        bins len[] = {[1:4]};
      }
      cp_beat_err : coverpoint f_beat_err iff (f_lnk_v) {
        bins clean        = {2'd0};
        bins corrupt_lcrc = {2'd1};
        bins dropped      = {2'd2};
        bins corrupt_data = {2'd3};
      }
      cx_len_err : cross cp_beat_len, cp_beat_err;

      // DLLPs seen by the transmitter
      cp_dllp : coverpoint f_dllp_nak iff (f_dllp_v) {
        bins ack = {1'b0};
        bins nak = {1'b1};
      }

      // replay causes
      cp_replay : coverpoint f_rep iff (f_rep != 2'b00) {
        bins by_nak     = {2'b01};
        bins by_timeout = {2'b10};
      }

      // sequence numbers, including both ends of the 12-bit range
      cp_seq : coverpoint f_seq iff (f_lnk_v) {
        bins seq_zero = {12'd0};
        bins seq_low  = {[12'd1    : 12'd15]};
        bins seq_mid  = {[12'd16   : 12'd4079]};
        bins seq_high = {[12'd4080 : 12'd4094]};
        bins seq_max  = {12'd4095};
      }

      // replay buffer occupancy
      cp_occ : coverpoint f_occ {
        bins empty = {4'd0};
        bins low   = {[4'd1 : 4'd3]};
        bins high  = {[4'd4 : 4'd7]};
        bins full  = {4'd8};
      }

      // sender stalled because the replay buffer is full
      cp_stall : coverpoint f_stall {
        bins stalled = {1'b1};
      }
    endgroup

    function new(string name, uvm_component parent);
      super.new(name, parent);
      cg_tlp  = new();
      cg_link = new();
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      if (!uvm_config_db #(virtual pcie_dll_if)::get(this, "", "vif", vif)) begin
        `uvm_fatal("COV", "virtual interface 'vif' not set")
      end
    endfunction

    // per accepted TLP
    function void write(pcie_tlp_item t);
      cg_tlp.sample(t);
    endfunction

    // every clock
    task run_phase(uvm_phase phase);
      forever begin
        @(vif.mon_cb);
        f_lnk_v    = vif.mon_cb.dbg_lnk_valid;
        f_lnk_len  = vif.mon_cb.dbg_lnk_len;
        f_beat_err = vif.mon_cb.dbg_beat_err;
        f_seq      = vif.mon_cb.dbg_lnk_seq;
        f_dllp_v   = vif.mon_cb.dbg_dllp_valid;
        f_dllp_nak = vif.mon_cb.dbg_dllp_is_nak;
        f_rep      = vif.mon_cb.dbg_replay_ev;
        f_occ      = vif.mon_cb.dbg_count;
        f_stall    = vif.mon_cb.in_valid && !vif.mon_cb.in_ready;
        cg_link.sample();
      end
    endtask

    function void report_phase(uvm_phase phase);
      real c_tlp, c_link;
      c_tlp  = cg_tlp.get_inst_coverage();
      c_link = cg_link.get_inst_coverage();
      `uvm_info("COV", $sformatf("FUNCTIONAL COVERAGE (overall)      = %0.2f %%", (c_tlp + c_link) / 2.0), UVM_NONE)
      `uvm_info("COV", $sformatf("  TLP group  (cg_tlp)               = %0.2f %%", c_tlp), UVM_NONE)
      `uvm_info("COV", $sformatf("  link group (cg_link)              = %0.2f %%", c_link), UVM_NONE)
      `uvm_info("COV", $sformatf("    TLP length                      = %0.2f %%", cg_tlp.cp_len.get_inst_coverage()), UVM_NONE)
      `uvm_info("COV", $sformatf("    link beat length                = %0.2f %%", cg_link.cp_beat_len.get_inst_coverage()), UVM_NONE)
      `uvm_info("COV", $sformatf("    link beat outcome               = %0.2f %%", cg_link.cp_beat_err.get_inst_coverage()), UVM_NONE)
      `uvm_info("COV", $sformatf("    cross: length x outcome         = %0.2f %%", cg_link.cx_len_err.get_inst_coverage()), UVM_NONE)
      `uvm_info("COV", $sformatf("    DLLP type (ACK/NAK)             = %0.2f %%", cg_link.cp_dllp.get_inst_coverage()), UVM_NONE)
      `uvm_info("COV", $sformatf("    replay cause (NAK/timeout)      = %0.2f %%", cg_link.cp_replay.get_inst_coverage()), UVM_NONE)
      `uvm_info("COV", $sformatf("    sequence number range / wrap    = %0.2f %%", cg_link.cp_seq.get_inst_coverage()), UVM_NONE)
      `uvm_info("COV", $sformatf("    replay buffer occupancy         = %0.2f %%", cg_link.cp_occ.get_inst_coverage()), UVM_NONE)
      `uvm_info("COV", $sformatf("    sender stalled (buffer full)    = %0.2f %%", cg_link.cp_stall.get_inst_coverage()), UVM_NONE)
    endfunction
  endclass

  //============================================================================
  // Environment
  //============================================================================
  class pcie_dll_env extends uvm_env;
    `uvm_component_utils(pcie_dll_env)

    pcie_dll_agent      agent;
    pcie_dll_err_inj    inj;
    pcie_dll_scoreboard scb;
    pcie_dll_coverage   cov;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      agent = pcie_dll_agent::type_id::create("agent", this);
      inj   = pcie_dll_err_inj::type_id::create("inj", this);
      scb   = pcie_dll_scoreboard::type_id::create("scb", this);
      cov   = pcie_dll_coverage::type_id::create("cov", this);
    endfunction

    function void connect_phase(uvm_phase phase);
      super.connect_phase(phase);
      agent.mon.ap_in.connect(scb.in_export);
      agent.mon.ap_out.connect(scb.out_export);
      agent.mon.ap_in.connect(cov.analysis_export);
    endfunction
  endclass

  //============================================================================
  // Test
  //   1. directed TLPs on a clean link
  //   2. saturated random traffic on a clean link (fills the replay buffer)
  //   3. one TLP is lost on an otherwise idle link  -> only the replay timer can recover it
  //   4. random traffic with moderate corruption and loss
  //   5. saturated random traffic with heavy corruption and loss
  //   6. clean link, drain
  //   +SCALE=n multiplies the amount of random traffic
  //============================================================================
  class pcie_dll_test extends uvm_test;
    `uvm_component_utils(pcie_dll_test)

    pcie_dll_env env;

    function new(string name, uvm_component parent);
      super.new(name, parent);
    endfunction

    function void build_phase(uvm_phase phase);
      super.build_phase(phase);
      env = pcie_dll_env::type_id::create("env", this);
    endfunction

    // wait until every accepted TLP has been delivered
    task drain();
      int unsigned t;
      #100ns;
      t = 0;
      while (env.scb.exp_q.size() != 0 && t < 20000) begin
        #10ns;
        t++;
      end
      if (env.scb.exp_q.size() != 0) begin
        `uvm_error("TEST", $sformatf("drain timeout: %0d TLPs still outstanding", env.scb.exp_q.size()))
      end
    endtask

    task run_rand(int unsigned n, bit b2b);
      pcie_dll_rand_seq rseq;
      rseq = pcie_dll_rand_seq::type_id::create("rseq");
      rseq.num_txn      = n;
      rseq.back_to_back = b2b;
      rseq.start(env.agent.sqr);
    endtask

    task run_phase(uvm_phase phase);
      pcie_dll_directed_seq dseq;
      int unsigned          scale;

      scale = 1;
      void'($value$plusargs("SCALE=%d", scale));
      if (scale == 0) scale = 1;

      phase.raise_objection(this);
      phase.phase_done.set_drain_time(this, 200ns);

      // 1. directed, clean link
      env.inj.corrupt_pct = 0;
      env.inj.drop_pct    = 0;
      dseq = pcie_dll_directed_seq::type_id::create("dseq");
      dseq.start(env.agent.sqr);
      drain();

      // 2. saturated random traffic, clean link
      run_rand(1000 * scale, 1'b1);
      drain();

      // 3. a single TLP is lost with nothing behind it
      env.inj.drop_next = 1'b1;
      run_rand(1, 1'b0);
      drain();

      // 4. moderate errors
      env.inj.corrupt_pct = 5;
      env.inj.drop_pct    = 5;
      run_rand(3000 * scale, 1'b0);
      drain();

      // 5. heavy errors, saturated sender
      env.inj.corrupt_pct = 10;
      env.inj.drop_pct    = 8;
      run_rand(2500 * scale, 1'b1);

      // 6. stop the errors and let everything drain
      env.inj.corrupt_pct = 0;
      env.inj.drop_pct    = 0;
      drain();

      phase.drop_objection(this);
    endtask
  endclass

endpackage

//==============================================================================
// Top module
//==============================================================================
module tb_top;
  import uvm_pkg::*;
  import pcie_dll_tb_pkg::*;

  logic ACLK    = 1'b0;
  logic ARESETn = 1'b0;

  always #5 ACLK = ~ACLK;                       // 100 MHz

  initial begin                                 // release reset after 5 clocks
    repeat (5) @(posedge ACLK);
    #1 ARESETn = 1'b1;
  end

  pcie_dll_if vif (.ACLK(ACLK), .ARESETn(ARESETn));

  pcie_dll_top dut (
    .clk(ACLK), .rst_n(ARESETn),
    .in_valid(vif.in_valid), .in_ready(vif.in_ready),
    .in_data(vif.in_data),   .in_len(vif.in_len),
    .out_valid(vif.out_valid), .out_data(vif.out_data), .out_len(vif.out_len),
    .inj_mode(vif.inj_mode),
    .dbg_lnk_valid(vif.dbg_lnk_valid), .dbg_lnk_seq(vif.dbg_lnk_seq),
    .dbg_lnk_len(vif.dbg_lnk_len),     .dbg_beat_err(vif.dbg_beat_err),
    .dbg_dllp_valid(vif.dbg_dllp_valid), .dbg_dllp_is_nak(vif.dbg_dllp_is_nak),
    .dbg_dllp_seq(vif.dbg_dllp_seq),
    .dbg_count(vif.dbg_count), .dbg_replay_ev(vif.dbg_replay_ev)
  );

  initial begin
    uvm_config_db #(virtual pcie_dll_if)::set(null, "*", "vif", vif);
    run_test("pcie_dll_test");
  end

  initial begin                                 // optional waveform: add +DUMP in Run Options
    if ($test$plusargs("DUMP")) begin
      $dumpfile("dump.vcd");
      $dumpvars(0);
    end
  end

  initial begin                                 // watchdog
    #50ms;
    $fatal(1, "TB timeout");
  end
endmodule
