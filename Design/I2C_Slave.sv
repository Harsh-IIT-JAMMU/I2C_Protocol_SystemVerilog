///////////////////// I2C SLAVE
`timescale 1ns / 1ps
 
// Slave = device that RESPONDS when master talks
module i2c_Slave(
    input scl, clk, rst,   // SCL = clock from master, clk = system clock
    inout sda,             // data line (shared with master)
    output reg ack_err,    // error if something goes wrong
    output reg done        // transaction completed
);
  
//------------------------------------------------------------
// States of slave (like steps in conversation)
//------------------------------------------------------------
typedef enum logic [3:0] {
    idle, read_addr, send_ack1, send_data,
    master_ack, read_data, send_ack2,
    wait_p, detect_stop
} state_type;

state_type state = idle;    
 
//------------------------------------------------------------
// Internal memory (like storage inside slave)
//------------------------------------------------------------
reg [7:0] mem [128];   // 128 locations, each 8-bit
reg [7:0] r_addr;      // received address
reg [6:0] addr;        // actual address
reg r_mem = 0;         // read enable
reg w_mem = 0;         // write enable
reg [7:0] dout;        // data to send
reg [7:0] din;         // data received
reg sda_t;             // value to drive on SDA
reg sda_en;            // control SDA direction
reg [3:0] bitcnt = 0;  // bit counter
 
//------------------------------------------------------------
// Initialize memory (just filling with values 0–127)
//------------------------------------------------------------
always@(posedge clk)
begin
  if(rst)
  begin
      for(int i = 0 ; i < 128; i++)
        mem[i] = i;   // simple pattern
      dout <= 8'h0;
  end
  else if (r_mem)
      dout <= mem[addr];   // read from memory
  else if (w_mem)
      mem[addr] <= din;    // write to memory
end
 
//------------------------------------------------------------
// Timing generation (same idea as master)
//------------------------------------------------------------
parameter sys_freq = 40000000;
parameter i2c_freq = 100000;
 
parameter clk_count4 = (sys_freq/i2c_freq);
parameter clk_count1 = clk_count4/4;

integer count1 = 0;
reg [1:0] pulse = 0;
reg busy;

// Generate timing pulses
always@(posedge clk)
begin
    if(rst)
    begin
        pulse <= 0;
        count1 <= 0;
    end
    else if(busy == 0)
    begin
        pulse <= 2;
        count1 <= 202; // idle alignment
    end
    else if(count1 == clk_count1 - 1) begin pulse <= 1; count1 <= count1 + 1; end
    else if(count1 == clk_count1*2 - 1) begin pulse <= 2; count1 <= count1 + 1; end
    else if(count1 == clk_count1*3 - 1) begin pulse <= 3; count1 <= count1 + 1; end
    else if(count1 == clk_count1*4 - 1) begin pulse <= 0; count1 <= 0; end
    else count1 <= count1 + 1;
end
 
//------------------------------------------------------------
// Detect START condition
// START = SDA goes LOW when SCL is HIGH
//------------------------------------------------------------
reg scl_t;
always@(posedge clk)
    scl_t <= scl;

wire start = ~scl & scl_t; // simple edge detect (approx)
 
reg r_ack;
 
//------------------------------------------------------------
// Main FSM (behavior of slave)
//------------------------------------------------------------
always@(posedge clk)
begin
if(rst)
begin
    bitcnt <= 0;
    state  <= idle;
    r_addr <= 0;
    sda_en <= 0;
    sda_t  <= 0;
    addr   <= 0;
    r_mem  <= 0;
    din    <= 0; 
    ack_err <= 0;
    done    <= 0;
    busy    <= 0;
end
 
else
begin
    case(state)
    
    //---------------- IDLE ----------------
    // Wait for master to start communication
    idle:
    begin
        if(scl == 1 && sda == 0) // start detected
        begin
            busy <= 1;
            state <= wait_p;
        end
    end
    
    //---------------- WAIT ----------------
    // Small delay to align timing
    wait_p:
    begin
        if (pulse == 3 && count1 == 399)
            state <= read_addr;
    end
    
    //---------------- READ ADDRESS ----------------
    read_addr:
    begin
        sda_en <= 0; // read from SDA
        if(bitcnt <= 7)
        begin
            if(pulse == 2)
                r_addr <= {r_addr[6:0], sda}; // shift bits
            
            if(count1 == clk_count1*4 - 1)
                bitcnt <= bitcnt + 1;
        end
        else
        begin
            state <= send_ack1;
            bitcnt <= 0;
            sda_en <= 1;
            addr <= r_addr[7:1]; // extract address
        end
    end
    
    //---------------- SEND ACK ----------------
    send_ack1:
    begin
        sda_t <= 0; // ACK = pull SDA low
        
        if(count1 == clk_count1*4 - 1)
        begin
            if(r_addr[0]) // read operation
            begin
                state <= send_data;
                r_mem <= 1;
            end
            else
            begin
                state <= read_data;
                r_mem <= 0;
            end
        end
    end
    
    //---------------- READ DATA FROM MASTER ----------------
    read_data:
    begin
        sda_en <= 0;
        if(bitcnt <= 7)
        begin
            if(pulse == 2)
                din <= {din[6:0], sda};
            
            if(count1 == clk_count1*4 - 1)
                bitcnt <= bitcnt + 1;
        end
        else
        begin
            state <= send_ack2;
            bitcnt <= 0;
            sda_en <= 1;
            w_mem <= 1; // store data
        end
    end
    
    //---------------- SEND ACK AFTER WRITE ----------------
    send_ack2:
    begin
        sda_t <= 0; // ACK
        
        if(count1 == clk_count1*4 - 1)
        begin
            state <= detect_stop;
            sda_en <= 0;
            w_mem <= 0;
        end
    end
    
    //---------------- SEND DATA TO MASTER ----------------
    send_data:
    begin
        sda_en <= 1;
        if(bitcnt <= 7)
        begin
            if(pulse == 1)
                sda_t <= dout[7-bitcnt]; // send bit
            
            if(count1 == clk_count1*4 - 1)
                bitcnt <= bitcnt + 1;
        end
        else
        begin
            state <= master_ack;
            bitcnt <= 0;
            sda_en <= 0;
        end
    end
    
    //---------------- MASTER ACK ----------------
    master_ack:
    begin
        if(pulse == 2)
            r_ack <= sda; // read master's response
        
        if(count1 == clk_count1*4 - 1)
        begin
            if(r_ack) // NACK
                ack_err <= 0;
            else
                ack_err <= 1;
            
            state <= detect_stop;
        end
    end
    
    //---------------- STOP DETECTION ----------------
    detect_stop:
    begin
        if(pulse == 3 && count1 == 399)
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
// SDA control (important concept)
//------------------------------------------------------------
// If enabled → slave drives SDA
// Else → releases line (master can drive)
assign sda = (sda_en) ? sda_t : 1'bz;
 
endmodule
