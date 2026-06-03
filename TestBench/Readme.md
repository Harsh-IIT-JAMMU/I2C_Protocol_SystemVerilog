Transaction

Think of this class as a package that carries all information needed
for one I2C communication.

For example, if the testbench wants to write data 5 to slave address 3,
all of that information is stored inside one transaction object.

The generator creates randomized transaction objects.
The driver reads the transaction and applies it to the DUT.
The monitor observes the DUT response.
The scoreboard checks whether the response is correct.

The constraints ensure that only valid and limited values are generated
during simulation. The 'op' field decides whether the transaction is a
READ or WRITE operation, and both are generated equally often (50%-50%).

In short, this class acts like a courier packet carrying all details
of a single I2C operation through different verification components.

Generator

The generator is the starting point of the verification flow.

Imagine you are testing a courier company. Before checking whether
packages are delivered correctly, someone must first create packages.
That is the generator's job.

The generator creates random I2C transactions such as:
- Write data 5 to address 3
- Read data from address 2
- Write data 8 to address 4

After creating a transaction, it places it into a mailbox.
The driver picks it up and applies it to the DUT.

The generator then waits for:
1. The driver to finish sending the transaction.
2. The scoreboard to finish checking the result.

Only after both are done does it create the next transaction.
This prevents different transactions from overlapping and keeps
the simulation organized.

When all requested transactions have been generated, the generator
triggers the 'done' event to tell the environment that its work
is complete.

Driver

The driver is the component that actually "presses the buttons"
of the DUT.

Imagine the generator is a manager giving instructions:

"Write value 7 to address 3."

The DUT cannot understand this instruction directly.
It only understands electrical signals such as:

newd = 1
op   = 0
addr = 3
din  = 7

The driver takes the transaction from the generator and
converts it into these interface signals.

For a WRITE operation:
1. Driver puts address and data on interface.
2. Driver asserts newd.
3. DUT starts I2C write transaction.
4. Driver waits until done signal becomes high.

For a READ operation:
1. Driver puts address on interface.
2. Driver asserts newd.
3. DUT reads data from slave.
4. Driver waits for done signal.
5. Driver prints received data.

After completing the operation, the driver triggers
the drvnext event so the generator knows it can
send the next transaction.



Monitor


The monitor acts like a CCTV camera connected to the DUT.

The driver actively sends commands to the DUT.
The monitor simply watches everything that happens.

For example, suppose the driver performs:

Write data 7 to address 3

The monitor observes:
- Which address was used
- Which data was sent
- Whether it was a read or write operation
- What data came back from the DUT

Once it captures this information, it creates a transaction
record and sends it to the scoreboard.

The scoreboard then checks whether the DUT behaved correctly.

Verification Flow:

Generator --> Driver --> DUT
                           |
                           v
                        Monitor
                           |
                           v
                       Scoreboard

The monitor is important because the scoreboard should verify
what actually happened on the interface, not what the generator
originally intended to send.







Scoreboard



Imagine you are testing an ATM machine.

Suppose you deposit ₹500 into account number 3.

The scoreboard keeps its own notebook and writes:

Account 3 = ₹500

Later, when the ATM says account 3 contains ₹500,
the scoreboard checks its notebook.

If both values match:
    PASS

If values differ:
    FAIL

The scoreboard works exactly the same way.

It maintains its own reference memory called mem[].
Whenever a WRITE operation occurs, it updates this memory.

Example:

WRITE:
Address = 4
Data = 8

Scoreboard updates:
mem[4] = 8

Later a READ occurs:

READ:
Address = 4

Expected value:
mem[4] = 8

If DUT returns 8:
    DATA MATCHED

If DUT returns anything else:
    DATA MISMATCHED

This is how the scoreboard determines whether the DUT
is functioning correctly.






1. Generator creates:
      Write Address=3 Data=7

2. Driver sends it to DUT

3. Monitor observes:
      Address=3 Data=7

4. Scoreboard updates:
      mem[3] = 7

------------------------------------------------

5. Generator creates:
      Read Address=3

6. Driver performs read

7. DUT returns:
      dout = 7

8. Monitor captures:
      Address=3 dout=7

9. Scoreboard checks:
      Expected = mem[3] = 7
      Received = 7

10. PASS





