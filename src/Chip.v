module Chip(
	//==Uart Interface
	output tx,
	input rx,

	//==Spi Interface
	output sck,	
	output mosi,
	input miso,

	//==Flash Interface
	output flash_sck,
    	output flash_cs,
    	output flash_mosi,
    	input flash_miso,

	//==Gpio Interface
	output [7:0] gpio_out,
	input [7:0] gpio_in,

	//==Sync Signals
	input clk,
	input reset_n
);

wire nreset;

ResetSync resetSync(
	.reset_raw( reset_n ),
	.nreset( nreset ),

	.clk( clk )
);

RiscyBoy cpu(
    .gpio_in( gpio_in ),
    .gpio_out( gpio_out ),

    .tx( tx ),
    .rx( rx ),

    .sck( sck ),
    .mosi( mosi ),
    .miso( miso ),

    .flash_sck( flash_sck ),
    .flash_cs( flash_cs ),
    .flash_mosi( flash_mosi ),
    .flash_miso( flash_miso ),

    .clk( clk ),
    .nreset( nreset )
);

endmodule



