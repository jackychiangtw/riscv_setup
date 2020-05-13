module ResetSync(
	input reset_raw,
	output nreset,

	input clk
);

reg [1:0] reset_sync;

always @ (posedge clk, negedge reset_raw)
begin
	if( !reset_raw ) begin
		reset_sync <= 2'b00;
	end
	else begin
		reset_sync <= {reset_sync[0], 1'b1};
	end
end

assign nreset = reset_sync[1];

endmodule
