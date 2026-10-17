PCIe Data Link Layer: Model and UVM Verification
A simplified PCIe Data Link Layer (DLL) written in SystemVerilog, with a UVM 1.2 environment that drives
traffic over a link that randomly corrupts and drops TLPs. The scoreboard checks that every TLP is
delivered in order, exactly once, whatever the link did to it.
What the DLL does
```
                      +---------------- pcie_dll_top -----------------+
 TLP in  ---------->  |  TX: seq number + LCRC          link           |
 (valid/ready)        |      replay buffer (8) -------> channel -----> RX: LCRC + seq check ----> TLP out
                      |      replay timer      (error injection)       |
                      |  <------- ACK / NAK DLLPs <-- 2-stage return path <---- ACK / NAK -------+
                      +-----------------------------------------------+
```
Transmitter (`pcie_dll_tx`)
Gives every TLP a 12-bit sequence number and a 32-bit LCRC and keeps it in an 8-entry replay buffer.
ACK(n) frees every TLP up to and including sequence number n (modulo 4096).
NAK(n) frees the acknowledged TLPs, then replays everything still unacknowledged, oldest first.
Replay timer: if nothing is acknowledged for 64 clocks, everything unacknowledged is replayed. This is the only
way to recover a TLP that was lost with nothing behind it.
The sender is stalled when the replay buffer is full.
Receiver (`pcie_dll_rx`)
Good LCRC and expected sequence number: deliver the TLP upward and return ACK.
Duplicate (already received): discard it and re-ACK the last good sequence number.
Bad LCRC or sequence-number gap: discard it and return NAK(last good). Only one NAK is sent until a good TLP is
received again (the PCIe NAK_SCHEDULED rule).
Link (`pcie_dll_channel`): per TLP, optionally flips an LCRC bit, flips a data bit, or drops the TLP.
Verification environment (`testbench.sv`)
Driver: offers TLPs of 1 to 4 double-words with random gaps, holding each one until it is accepted.
Error injector: every clock, randomly corrupts or drops the TLP on the link (percentages set per test phase),
and can drop exactly one chosen TLP.
Monitor: reports TLPs accepted by the sender, TLPs delivered by the receiver, and link-level event counters.
Scoreboard: every accepted TLP must be delivered exactly once, in order, with unchanged contents. It also
fails if anything is delivered with none outstanding (duplicate) or if anything is left undelivered.
Coverage collector: see below.
Test phases:
Directed TLPs of every length and corner payloads, on a clean link.
1000 back-to-back random TLPs on a clean link (fills the replay buffer).
One TLP lost on an idle link, so only the replay timer can recover it.
3000 random TLPs with 5 % corruption and 5 % loss.
2500 back-to-back random TLPs with 10 % corruption and 8 % loss.
Errors off, drain.
`+SCALE=n` multiplies the amount of random traffic, and `+DUMP` writes a waveform.
Assertions and cover properties (in `pcie_dll_if`)
Assertions (8): TLP held until accepted; legal input length; legal output length; replay buffer never overflows;
full buffer stalls the sender; a replay only happens with unacknowledged data; no unknown values on control
signals; no unknown DLLP fields.
Cover properties (6): NAK returned, replay caused by NAK, replay caused by timeout, replay buffer full, sender
stalled, sequence number wraparound (4095 followed by 0).
Functional coverage
`cg_tlp`: TLP length.
`cg_link` (sampled every clock): link beat length, what the link did to each beat (clean, corrupted LCRC, dropped,
corrupted data), the cross of length x outcome, DLLP type, replay cause (NAK or timeout), sequence-number
range including 0 and 4095, replay buffer occupancy, and sender stalled with a full buffer.
Results (Synopsys VCS X-2025.06-SP1, UVM 1.2, via EDA Playground)
First run, using the testbench before the length fix described below:
Item	Result
Scoreboard	6,517 TLPs accepted and delivered in order, exactly once, 0 mismatches
Link activity	11,081 link beats; 772 corrupted and 639 dropped
Replays	718 caused by NAK, 125 caused by the replay timer
UVM errors / fatals	0 / 0

Cover properties	6 of 6 hit
Functional coverage	96.48 % overall (100 % TLP group, 92.97 % link group)
The raw log lines are in `results/vcs_run_summary.txt`.
Coverage finding and fix
The length x outcome cross was at 43.75 % (7 of 16 bins) even though 1,411 errors were injected. Random TLPs were
almost always 4 double-words long: the solver weighs every (length, data) pair equally, and a 4-double-word TLP has
far more data values than a shorter one. Lengths 1 to 3 appeared only in the directed phase, so errors were
never injected on short TLPs. `testbench.sv` now has `solve len before data;` in the item constraints so the
length is chosen first and is uniform.
Run it
Open EDA Playground.
Language: SystemVerilog/Verilog. Library: UVM 1.2. Simulator: Synopsys VCS (or Xcelium, Questa or Riviera-PRO).
Leave SVUnit, Easier UVM and TL-Verilog switched off.
Paste `testbench.sv` into the Testbench pane and `design.sv` into the Design pane, then run.
Search the log for `PASS`, `LINK` and `COV`.
Python reference model
`model/pcie_dll_model.py` is a cycle-by-cycle Python mirror of the transmitter, link and receiver. It was used to check
the algorithm (acknowledgement arithmetic, replay, replay timer, NAK_SCHEDULED rule) under random loss and
corruption before the SystemVerilog was run, and it passes with every sequence delivered in order. With the replay
timer disabled it deadlocks, which confirms the model detects that bug.
```
python3 model/pcie_dll_model.py 20      # 20 random seeds
```
Files
```
design.sv                     TX, RX, error-injecting link, top level, interface with assertions and covers
testbench.sv                  UVM environment, sequences, coverage, top module
model/pcie_dll_model.py       Python cycle model of the protocol
results/vcs_run_summary.txt   result lines copied from the VCS run
```
Simplifications
This is a model of the Data Link Layer's behaviour, not a PCIe-compliant implementation.
Each TLP crosses the link as one wide beat of up to 4 double-words; there are no framing symbols.
The LCRC uses the CRC-32 polynomial 0x04C11DB7 but not PCIe's exact bit ordering and inversion.
DLLPs carry no CRC-16, are never corrupted or lost, and every good TLP is acknowledged individually.
No flow-control DLLPs, no power-management DLLPs, no link training.
Verified in simulation only.
