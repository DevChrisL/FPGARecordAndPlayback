`timescale 1ns / 1ps


module audio_device(

	
	
	

	// switches to control the board
	input	[7:0]	switches,
	//leds to show addresses accessed by ram
	output	[7:0]	leds,
	// pico uart lines
	input			rs232_rx,
	output			rs232_tx,
	input [3:0] button,
	


    inout  AUD_ADCLRCK,
    input  AUD_ADCDAT,

    inout  AUD_DACLRCK,
    output AUD_DACDAT,

    output AUD_XCK,
    inout  AUD_BCLK,

    output AUD_I2C_SCLK,
    inout  AUD_I2C_SDAT,

    output AUD_MUTE,
	 output PLL_LOCKED,

    output [2:0] LED,
	 

	output 	hw_ram_rasn,
	output 	hw_ram_casn,
	output 	hw_ram_wen,
	output[2:0] hw_ram_ba,
	inout 	hw_ram_udqs_p,
	inout 	hw_ram_udqs_n,
	inout 	hw_ram_ldqs_p,
	inout 	hw_ram_ldqs_n,
	output 	hw_ram_udm,
	output 	hw_ram_ldm,
	output 	hw_ram_ck,
	output 	hw_ram_ckn,
	output 	hw_ram_cke,
	output 	hw_ram_odt,
	output[12:0] hw_ram_ad,
	inout [15:0] hw_ram_dq,
	inout 	hw_rzq_pin,
	inout 	hw_zio_pin,
	output ledRAM,
	
	
	input			reset,			// Enable
	input			reset_clock,
	input			clk,			// 100 MHz
	
	input pause,
	input clear,
	input address_switch

	
	);
		

	
	wire	[7:0]	pb_port_id;
	wire	[7:0]	pb_out_port;
	reg		[7:0]	pb_in_port;
	wire			pb_read_strobe;
	wire			pb_write_strobe;

	wire			pb_reset;
	wire			pb_interrupt;
	wire			pb_int_ack;
	
	
	wire			write_to_uart;
	wire			uart_buffer_full;
	wire			uart_data_present;
	reg				read_from_uart;
	wire			uart_reset;
	
	
	wire	[7:0]	uart_rx_data;
	

	wire led_write;
	wire led_reset;

	wire main_clk;
	wire ram_clk;
	wire audio_clk;

	wire [1:0] sample_end;
	wire [1:0] sample_req;

	wire [15:0] audio_input;
	reg [15:0] audio_output;
		 
	wire 	systemCLK;

clk_wiz_v3_6 pll (
	 .CLK_IN1 (clk_100MHz),
	 .CLK_OUT1 (main_clk),   // 50 MHz
    .CLK_OUT2 (audio_clk),  // 11.2896 MHz
	 .RESET (reset_clock),
	 .LOCKED (PLL_LOCKED)
);

//clk generator
clkgen clock_generator (
	 .CLK_IN1 (systemCLK),
	 .CLK_OUT1 (clk_100MHz)    // 100 MHz	 unbuffered

);

	reg 	[25:0] address = 0; 
	reg 	[15:0]	RAMin;
	wire 	[15:0]	RAMout;
	reg	[7:0] dataOut; 
	reg 			reqRead;
	reg 			enableWrite;
	reg 			ackRead = 0;

	
	wire rdy, 	dataPresent;
	wire [25:0]	max_ram_address;
	
	reg [3:0]	state=4'b0000;
	
	parameter stInit = 4'b0000;
	parameter stReadFromPorts = 3'b001;
	parameter stMemWrite = 3'b0010;
	parameter stMemReadReq  = 3'b0011;
	parameter stMemReadData = 3'b0100;
	
	reg ram_write_request;
	reg ram_read_request;
	reg ram_write_ack;
	reg ram_read_ack;
	reg [15:0] audio_input_sample;
	reg [15:0] audio_output_sample;
	reg [15:0] debug_sample;
		

	
	always @(posedge systemCLK)
	begin
		if (reset) begin 
			if(address_switch == 1'b0)begin
				address <= 0;
			end 
			else if(address_switch ==1'b1)begin
				address <= 26'b00110000000000000000000000;
			end
			
			state <= stInit;
			
		end
		else
			if(rdy) begin // if ram ready
				case (state)
				  // Initialization state
				  stInit: begin 
				   ackRead <= 1'b0;
					enableWrite <= 1'b0;
					
					if(switches[2]) begin  //reset address to 0
					
						if(address_switch == 1'b0)begin
							address <= 0;
						end 
						else if(address_switch == 1'b1)begin
							address <= 26'b00110000000000000000000000;
						end

					end
					
					else begin
						address <= address;
					end
					
					if(!switches[0] & switches[1]) begin //RECORD
						if (address == max_ram_address) begin
							
						end
						else begin
							address <= address + 1'b1;
							
						end
						state <= stReadFromPorts;
					end
					else if (switches[0] & !switches[1]) begin //PLAY
						if (address == max_ram_address) begin
							
						end
						else begin
							if(pause == 1'b1)begin		//PAUSE
								address <= address;
							end
							else begin
								address <= address + 1'b1;		//dont pause
								
							end
						end
						state <= stMemReadReq;
					end
					else begin
						state <= stInit;
					end
					end

				  stReadFromPorts: begin
					if(sample_end) begin
						if(!clear)begin
							RAMin <= audio_input_sample;
							state <= stMemWrite;
						end
						else begin
							RAMin <= 16'b0000000000000000;
							state <= stMemWrite;
						end
					end
					else begin
						state <= stReadFromPorts;
					end
				  end
				  
				  
				  stMemWrite: begin
				   enableWrite <= 1'b1;
					state <= stInit;
				  end
				  
				  // Read cycle 1, pull down write enable, raise read request
				  stMemReadReq: begin
				  if(sample_end) begin
					enableWrite <= 1'b0;
					reqRead <= 1'b1;
					state <= stMemReadData;
				  end
				  else begin
						state <= stMemReadReq;
					end
				end
				  
				  // Read cycle 2
				  // Waite until data is valid i.e., when dataPresent is 1
				  stMemReadData: begin
					reqRead <= 1'b0;
					if(dataPresent) begin // data is present, read to dataOut register
						audio_output_sample = RAMout;
						ackRead <= 1'b1;	 // acknowledge the read
						state <= stInit;
					end
					else begin				  // stay in the same state until data is valid
						state <= stMemReadData;
					end
				  end
			 
			 endcase 
			end // rdy
		end

always @(posedge clk_100MHz) begin
    if (sample_end) begin
        audio_input_sample <= audio_input;
    end

end



	// ram interface instantiation
	ram_interface_wrapper RAMRapper (
					.address(address),				// input 
					.data_in(RAMin), 					// input
					.write_enable(enableWrite), 	//	input
					.read_request(reqRead), 		//	input
					.read_ack(ackRead), 
					.data_out(RAMout), 				// output from ram to wire
					.reset(reset_1), 
					.clk(clk), //clk_100MHz
					.hw_ram_rasn(hw_ram_rasn), 
					.hw_ram_casn(hw_ram_casn),
					.hw_ram_wen(hw_ram_wen), 
					.hw_ram_ba(hw_ram_ba), 
					.hw_ram_udqs_p(hw_ram_udqs_p), 
					.hw_ram_udqs_n(hw_ram_udqs_n), 
					.hw_ram_ldqs_p(hw_ram_ldqs_p), 
					.hw_ram_ldqs_n(hw_ram_ldqs_n), 
					.hw_ram_udm(hw_ram_udm), 
					.hw_ram_ldm(hw_ram_ldm), 
					.hw_ram_ck(hw_ram_ck), 
					.hw_ram_ckn(hw_ram_ckn), 
					.hw_ram_cke(hw_ram_cke), 
					.hw_ram_odt(hw_ram_odt),
					.hw_ram_ad(hw_ram_ad), 
					.hw_ram_dq(hw_ram_dq), 
					.hw_rzq_pin(hw_rzq_pin), 
					.hw_zio_pin(hw_zio_pin), 
					.clkout(systemCLK), 
					.sys_clk(systemCLK), 
					.rdy(rdy), 
					.rd_data_pres(dataPresent),
					.max_ram_address(max_ram_address),
					.ledRAM(ledRAM)
					);		
					
assign leds = address[23:16]; // output data read to leds
assign LED = state;


// I2C Protocol - FPGA is Master, Codec is Slave
i2c_av_config av_config (
    .clk (main_clk),
    .reset (reset_1),
    .i2c_sclk (AUD_I2C_SCLK),
    .i2c_sdat (AUD_I2C_SDAT)//,
 //   .status (LED)
);

assign AUD_XCK = audio_clk;

assign AUD_MUTE = switches[0];  // active low, so set to 1 and disable mute

// Serial to parallel conversion 
audio_codec ac (
    .clk (audio_clk),
    .reset (reset_1),
    .sample_end (sample_end),
    .sample_req (sample_req),
 //   .audio_output (audio_output),
    .audio_output (RAMout),
    .audio_input (audio_input),
    .channel_sel (2'b10),

    .AUD_ADCLRCK (AUD_ADCLRCK),
    .AUD_ADCDAT (AUD_ADCDAT),
    .AUD_DACLRCK (AUD_DACLRCK),
    .AUD_DACDAT (AUD_DACDAT),
    .AUD_BCLK (AUD_BCLK)
);

assign reset_1 = reset_clock;
	
	// UART and control logic
	//
	// UART expects ACTIVE-HIGH reset	
	assign uart_reset =  reset_1;
	// UART instantiation
	//
	// Within the UART Module (rs232_uart.v), make sure you fill in the
	// appropriate sections.
	rs232_uart UART (
		.tx_data_in(pb_out_port), // The UART only accepts data from PB, so we just tie the PB output to the UART input.
		.write_tx_data(write_to_uart), // Goes high when PB sends write strobe and PORT_ID is the UART write port number
		.tx_buffer_full(uart_buffer_full),
		.rx_data_out(uart_rx_data),
		.read_rx_data_ack(read_from_uart),
		.rx_data_present(uart_data_present),
		.rs232_tx(rs232_tx),
		.rs232_rx(rs232_rx),
		.reset(uart_reset),
		.clk(clk_100MHz) //main_clk
	);	
	
	// PicoBlaze and control logic
	//
	// PB expects ACTIVE-HIGH reset (changed from ~reset)
	assign pb_reset = reset_1;
	// Disable interrupt by assigning 0 to interrupt
	assign pb_interrupt = 1'b0;
	// PB CPU instantiation
	//
	// Within the PicoBlaze Module (picoblaze.v), make sure you fill in the
	// appropriate sections.
	picoblaze CPU (
		.port_id(pb_port_id),
		.read_strobe(pb_read_strobe),
		.in_port(pb_in_port),
		.write_strobe(pb_write_strobe),
		.out_port(pb_out_port),
		.interrupt(pb_interrupt),
		.interrupt_ack(),
		.reset(pb_reset),
		.clk(clk_100MHz) //main clk
	);	
	// PB I/O selection/routing
	//
	// Handle PicoBlaze Output Port Logic
	// Output Ports:
	// * leds_out : port 01
	// * uart_data_tx : port 03
	//
	// These signals are effectively "write enable" lines for the UART and LED
	// Driver modules. They must be asserted when PB is outputting to the
	// corresponding port
	assign led_write = pb_write_strobe & (pb_port_id == 8'h01);
	assign write_to_uart = pb_write_strobe & (pb_port_id == 8'h03);
	//
	// Handle PicoBlaze Input Port Logic
	// Input Ports:
	// * switches_in : port 00
	// * uart_data_rx : port 02
	// * uart_data_present : port 04 (1-bit, assigned to LSB)
	// * uart_buffer_full: port 05 (1-bit, assigned to LSB)
	//
	// This process block gets the value of the requested input port device
	// and passes it to PBs in_port. When PB is not requestng data from
	// a valid input port, set the input to static 0.
	always @(posedge clk_100MHz or posedge pb_reset)
	begin
		if(pb_reset) begin
			pb_in_port <= 0;
			read_from_uart <= 0;
		end else begin
			// Set pb input port to appropriate value
			case(pb_port_id)
				8'h00: pb_in_port <= switches;
				8'h02: pb_in_port <= uart_rx_data;
				8'h04: pb_in_port <= {7'b0000000,uart_data_present};
				8'h05: pb_in_port <= {7'b0000000,uart_buffer_full};
				default: pb_in_port <= 8'h00;
			endcase
			// Set up acknowledge/enable signals.
			//
			// Some modules, such as the UART, need confirmation that the data
			// has been read, since it needs to push it off the queue and make
			// the next byte available. This logic will set the 'read_from'
			// signal high for corresponding ports, as needed. Most input
			// ports will not need this.
			read_from_uart <= pb_read_strobe & (pb_port_id == 8'h04);
		end
	end

endmodule
