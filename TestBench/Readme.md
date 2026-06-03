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
