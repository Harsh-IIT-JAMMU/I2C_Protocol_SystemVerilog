# I2C Verification Testbench in SystemVerilog

## Overview

This project implements a transaction-based SystemVerilog verification environment for an I2C design. The testbench follows a modular architecture consisting of a Generator, Driver, Monitor, and Scoreboard communicating through mailboxes and synchronized using events.

The objective of the testbench is to automatically generate random I2C read and write transactions, apply them to the DUT, monitor the DUT behavior, and verify correctness using a reference model.

---

# Verification Architecture

```text
                 mailbox (mbxgd)

 Generator ---------------------> Driver
     ^                              |
     |                              |
     | drvnext event                |
     |                              v
     |                           +------+
     |                           | DUT  |
     |                           +------+
     |                              |
     |                              v
     |                         Monitor
     |                              |
     | mailbox (mbxms)              |
     +------------------------------+
                                    |
                                    v
                               Scoreboard
                                    |
                                    |
                               sconext event
                                    |
                                    +------> Generator
```

---

# Verification Flow

1. Generator creates randomized transactions.
2. Driver receives transactions and converts them into DUT interface signals.
3. DUT performs the requested I2C operation.
4. Monitor observes DUT activity and captures transaction information.
5. Scoreboard compares DUT results against a reference model.
6. Generator waits for both Driver and Scoreboard to complete before creating the next transaction.
7. After all transactions are processed, simulation terminates.

---

# Components

## Transaction

The transaction class represents a single I2C operation.

Think of this class as a package that carries all information needed for one I2C communication.

For example, if the testbench wants to write data 5 to slave address 3, all of that information is stored inside one transaction object.

The generator creates randomized transaction objects. The driver reads the transaction and applies it to the DUT. The monitor observes the DUT response. The scoreboard checks whether the response is correct.

The constraints ensure that only valid and limited values are generated during simulation.

### Transaction Fields

| Field   | Description                  |
| ------- | ---------------------------- |
| op      | Read/Write operation         |
| addr    | 7-bit slave address          |
| din     | Data to be written           |
| dout    | Data returned by DUT         |
| busy    | DUT busy status              |
| done    | Transaction completion flag  |
| ack_err | Acknowledge error indication |

### Constraints

* Address values generated between 2 and 4
* Data values generated between 2 and 9
* Read and Write operations generated with equal probability (50%-50%)

In short, this class acts like a courier packet carrying all details of a single I2C operation through different verification components.

---

## Generator

The generator is the starting point of the verification flow.

Imagine you are testing a courier company. Before checking whether packages are delivered correctly, someone must first create packages. That is the generator's job.

The generator creates random I2C transactions such as:

* Write data 5 to address 3
* Read data from address 2
* Write data 8 to address 4

After creating a transaction, it places it into a mailbox. The driver picks it up and applies it to the DUT.

The generator then waits for:

1. The driver to finish sending the transaction.
2. The scoreboard to finish checking the result.

Only after both are done does it create the next transaction. This prevents different transactions from overlapping and keeps the simulation organized.

When all requested transactions have been generated, the generator triggers the `done` event to tell the environment that its work is complete.

---

## Driver

The driver is the component that actually applies stimulus to the DUT.

Imagine the generator is a manager giving instructions:

> "Write value 7 to address 3."

The DUT cannot understand this instruction directly. It only understands electrical signals such as:

```text
newd = 1
op   = 0
addr = 3
din  = 7
```

The driver takes the transaction from the generator and converts it into these interface signals.

### For a WRITE operation

1. Driver places address and data on interface.
2. Driver asserts `newd`.
3. DUT starts I2C write transaction.
4. Driver waits for `done`.

### For a READ operation

1. Driver places address on interface.
2. Driver asserts `newd`.
3. DUT performs read operation.
4. Driver waits for `done`.
5. Driver captures returned data.

After completing the operation, the driver triggers the `drvnext` event so the generator knows it can send the next transaction.

---

## Monitor

The monitor acts like a CCTV camera connected to the DUT.

The driver actively sends commands to the DUT. The monitor simply watches everything that happens.

For example, suppose the driver performs:

```text
Write data 7 to address 3
```

The monitor observes:

* Which address was used
* Which data was sent
* Whether it was a read or write operation
* What data came back from the DUT

Once it captures this information, it creates a transaction record and sends it to the scoreboard.

The scoreboard then checks whether the DUT behaved correctly.

### Verification Path

```text
Generator --> Driver --> DUT
                           |
                           v
                        Monitor
                           |
                           v
                       Scoreboard
```

The monitor is important because the scoreboard should verify what actually happened on the interface, not what the generator originally intended to send.

---

## Scoreboard

The scoreboard is responsible for checking correctness of DUT behavior.

Imagine you are testing an ATM machine.

Suppose you deposit ₹500 into account number 3.

The scoreboard keeps its own notebook and writes:

```text
Account 3 = ₹500
```

Later, when the ATM says account 3 contains ₹500, the scoreboard checks its notebook.

If both values match:

```text
PASS
```

If values differ:

```text
FAIL
```

The scoreboard works exactly the same way.

It maintains its own reference memory called `mem[]`.

Whenever a WRITE operation occurs, it updates this memory.

### Example

WRITE:

```text
Address = 4
Data    = 8
```

Scoreboard updates:

```text
mem[4] = 8
```

Later a READ occurs:

```text
Address = 4
```

Expected value:

```text
mem[4] = 8
```

If DUT returns:

```text
8
```

Result:

```text
DATA MATCHED
```

If DUT returns anything else:

```text
DATA MISMATCHED
```

This is how the scoreboard determines whether the DUT is functioning correctly.

---

# Example Transaction Flow

### Write Transaction

Generator creates:

```text
Write Address = 3
Data = 7
```

Driver sends it to DUT.

Monitor observes:

```text
Address = 3
Data = 7
```

Scoreboard updates:

```text
mem[3] = 7
```

---

### Read Transaction

Generator creates:

```text
Read Address = 3
```

Driver performs read operation.

DUT returns:

```text
dout = 7
```

Monitor captures:

```text
Address = 3
dout = 7
```

Scoreboard checks:

```text
Expected = mem[3] = 7
Received = 7
```

Result:

```text
PASS
```

---

# Features

* Transaction-based verification
* Randomized stimulus generation
* Event-based synchronization
* Mailbox communication
* Self-checking scoreboard
* Read and Write transaction support
* Reference memory model
* Waveform generation using VCD dump
* Modular and reusable architecture

---

# Simulation

The testbench generates a configurable number of randomized transactions.

Example:

```systemverilog
gen.count = 20;
```

This generates 20 random I2C transactions and verifies DUT functionality automatically.

---

# Future Improvements

* Functional Coverage
* Constrained Random Verification Enhancements
* Assertions (SVA)
* Error Injection Scenarios
* UVM Migration
* Multiple Slave Support
* Protocol Compliance Checks

---
