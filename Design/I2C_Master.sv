`timescale 1ns / 1ps
 
// I2C Master: This block controls the communication (it "talks first")
module i2c_master(
    input clk, rst, newd,       // clock, reset, and "new data" signal
    input [6:0] addr,           // 7-bit slave address
    input op,                   // operation: 0 = write, 1 = read
    inout sda,                  // I2C data line (bi-directional)
    output scl,                 // I2C clock line
    input [7:0] din,            // data to send
    output [7:0] dout,          // data received
    output reg busy, ack_err, done // status signals
);
 
// Internal signals to control SDA and SCL
reg scl_t = 0;
reg sda_t = 0;
 
// System clock = 40 MHz, I2C clock = 100 kHz
parameter sys_freq = 40000000;
parameter i2c_freq = 100000;
 
// Clock division to generate slower I2C timing
parameter clk_count4 = (sys_freq/i2c_freq); // total count
parameter clk_count1 = clk_count4/4;        // quarter cycle
 
integer count1 = 0;
reg [1:0] pulse = 0; // divides each I2C bit into 4 small steps
 
//------------------------------------------------------------
// Generate timing pulses (used to control SCL and SDA timing)
//------------------------------------------------------------
always@(posedge clk)
begin
    if(rst)
    begin
        pulse <= 0;
        count1 <= 0;
    end
    else if (busy == 1'b0) // only run when transaction starts
    begin
        pulse <= 0;
        count1 <= 0;
    end
    else if(count1 == clk_count1 - 1) begin pulse <= 1; count1 <= count1 + 1; end
    else if(count1 == clk_count1*2 - 1) begin pulse <= 2; count1 <= count1 + 1; end
    else if(count1 == clk_count1*3 - 1) begin pulse <= 3; count1 <= count1 + 1; end
    else if(count1 == clk_count1*4 - 1) begin pulse <= 0; count1 <= 0; end
    else count1 <= count1 + 1;
end
 
//------------------------------------------------------------
// Internal registers
//------------------------------------------------------------
reg [3:0] bitcount = 0;        // counts bits (0 to 7)
reg [7:0] data_addr = 0;       // address + R/W bit
reg [7:0] data_tx = 0;         // data to send
reg [7:0] rx_data = 0;         // data received
reg r_ack = 0;                 // stores ACK from slave
reg sda_en = 0;                // controls SDA direction (read/write)
 
//------------------------------------------------------------
// State Machine (brain of I2C master)
//------------------------------------------------------------
typedef enum logic [3:0] {
    idle, start, write_addr, ack_1,
    write_data, read_data, stop,
    ack_2, master_ack
} state_type;

state_type state = idle;
 
//------------------------------------------------------------
// Main FSM logic
//------------------------------------------------------------
always@(posedge clk)
begin
 if(rst)
 begin
    // Reset everything
    bitcount <= 0;
    data_addr <= 0;
    data_tx <= 0;
    scl_t <= 1;
    sda_t <= 1;
    state <= idle;
    busy <= 0;
    ack_err <= 0;
    done <= 0;
 end
 else
 begin
    case(state)
    
    //---------------- IDLE ----------------
    // Waiting for new transaction
    idle:
    begin
        done <= 0;
        if(newd)
        begin
            data_addr <= {addr, op}; // combine address + read/write bit
            data_tx <= din;
            busy <= 1;
            state <= start;
            ack_err <= 0;
        end
        else
        begin
            busy <= 0;
            state <= idle;
        end
    end
    
    //---------------- START CONDITION ----------------
    // SDA goes LOW while SCL is HIGH
    start:
    begin
        sda_en <= 1;
        case(pulse)
            0,1: begin scl_t <= 1; sda_t <= 1; end
            2,3: begin scl_t <= 1; sda_t <= 0; end // start condition
        endcase
        
        if(count1 == clk_count1*4 - 1)
        begin
            state <= write_addr;
            scl_t <= 0;
        end
    end
    
    //---------------- SEND ADDRESS ----------------
    write_addr:
    begin
        sda_en <= 1;
        if(bitcount <= 7)
        begin
            case(pulse)
                0: begin scl_t <= 0; end
                1: begin scl_t <= 0; sda_t <= data_addr[7-bitcount]; end
                2,3: begin scl_t <= 1; end
            endcase
            
            if(count1 == clk_count1*4 - 1)
            begin
                bitcount <= bitcount + 1;
            end
        end
        else
        begin
            state <= ack_1; // wait for ACK from slave
            bitcount <= 0;
            sda_en <= 0;
        end
    end
    
    //---------------- ACK AFTER ADDRESS ----------------
    ack_1:
    begin
        sda_en <= 0; // read from slave
        case(pulse)
            2: begin scl_t <= 1; r_ack <= sda; end
            default: scl_t <= 0;
        endcase
        
        if(count1 == clk_count1*4 - 1)
        begin
            if(r_ack == 0 && data_addr[0] == 0)
                state <= write_data; // write mode
            else if(r_ack == 0 && data_addr[0] == 1)
                state <= read_data;  // read mode
            else
            begin
                state <= stop; // error
                ack_err <= 1;
            end
        end
    end
    
    //---------------- WRITE DATA ----------------
    write_data:
    begin
        if(bitcount <= 7)
        begin
            case(pulse)
                1: sda_t <= data_tx[7-bitcount];
                2,3: scl_t <= 1;
                default: scl_t <= 0;
            endcase
            
            if(count1 == clk_count1*4 - 1)
                bitcount <= bitcount + 1;
        end
        else
        begin
            state <= ack_2; // wait for ACK
            bitcount <= 0;
            sda_en <= 0;
        end
    end
    
    //---------------- READ DATA ----------------
    read_data:
    begin
        sda_en <= 0;
        if(bitcount <= 7)
        begin
            if(pulse == 2)
                rx_data <= {rx_data[6:0], sda}; // shift in data
            
            if(count1 == clk_count1*4 - 1)
                bitcount <= bitcount + 1;
        end
        else
        begin
            state <= master_ack; // send ACK/NACK
            bitcount <= 0;
            sda_en <= 1;
        end
    end
    
    //---------------- MASTER ACK ----------------
    master_ack:
    begin
        sda_en <= 1;
        sda_t <= 1; // sending NACK (end of read)
        
        if(count1 == clk_count1*4 - 1)
            state <= stop;
    end
    
    //---------------- ACK AFTER DATA ----------------
    ack_2:
    begin
        sda_en <= 0;
        if(pulse == 2)
            r_ack <= sda;
        
        if(count1 == clk_count1*4 - 1)
        begin
            state <= stop;
            ack_err <= (r_ack != 0);
        end
    end
    
    //---------------- STOP CONDITION ----------------
    // SDA goes HIGH while SCL is HIGH
    stop:
    begin
        sda_en <= 1;
        case(pulse)
            0,1: begin scl_t <= 1; sda_t <= 0; end
            2,3: begin scl_t <= 1; sda_t <= 1; end // stop condition
        endcase
        
        if(count1 == clk_count1*4 - 1)
        begin
            state <= idle;
            busy <= 0;
            done <= 1;
        end
    end
    
    default: state <= idle;
    
    endcase
 end
end
 
//------------------------------------------------------------
// SDA line control (important concept in I2C)
//------------------------------------------------------------
// If enabled → master drives SDA
// If disabled → line is released (slave can drive it)
assign sda = (sda_en) ? (sda_t ? 1'b1 : 1'b0) : 1'bz;
 
// Output assignments
assign scl = scl_t;
assign dout = rx_data;
 
endmodule
