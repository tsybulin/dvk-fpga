module ws2812(
    input               clk,
    input       [4:0]   led,
    output reg          led2812
) ;

    localparam WS2812_WIDTH = 120 ;

    localparam RED      = 24'h001000 ;
    localparam GREEN    = 24'h000008;
    localparam BLUE     = 24'h200000 ;
    localparam YELLOW   = 24'h000808 ;
    localparam WHITE    = 24'h080808 ;
    localparam BLACK    = 24'd0 ;
    
    wire [119:0] led_data = {
        led[4] ? BLUE   : BLACK,
        led[3] ? YELLOW : BLACK,
        led[2] ? YELLOW : BLACK,
        led[1] ? WHITE  : BLACK,
        led[0] ? GREEN  : BLACK
    } ;

    localparam CLK_FRE = 50_000_000 ;
    localparam DELAY_RESET = 13999; // (CLK_FRE / 1_000_000 * 280 ) - 1 ;
    localparam DELAY_1_HIGH = 41 ; // (CLK_FRE / 1_000_000 * 0.85 ) - 1 ;
    localparam DELAY_0_HIGH = 19 ; // (CLK_FRE / 1_000_000 * 0.40 ) - 1 ;
    localparam DELAY_1_LOW = 19 ; // (CLK_FRE / 1_000_000 * 0.40 ) - 1 ;
    localparam DELAY_0_LOW = 41 ; // (CLK_FRE / 1_000_000 * 0.85 ) - 1 ;

    localparam WSS_RESET            = 2'd0 ;
    localparam WSS_DATA_SEND        = 2'd1 ;
    localparam WSS_BIT_SEND_HIGH    = 2'd2 ;
    localparam WSS_BIT_SEND_LOW     = 2'd3 ;

    reg [1:0] ws_state = WSS_RESET ;
    reg [31:0] counter = 0 ;
    reg [7:0] bit_send = 0 ;

    always @(posedge clk) begin
        case (ws_state)
            WSS_RESET : begin
                led2812 <= 1'b0 ;
                if (counter < DELAY_RESET)
                    counter <= counter + 1'd1 ;
                else begin
                    counter <= 0 ;
                    ws_state <= WSS_DATA_SEND ;
                end
            end

            WSS_DATA_SEND : begin
                if (bit_send == WS2812_WIDTH) begin
                    counter <= 0 ;
                    bit_send <= 0 ;
                    ws_state <= WSS_RESET ;
                end else if (bit_send < WS2812_WIDTH) begin
                    ws_state <= WSS_BIT_SEND_HIGH ;
                end else begin
                    bit_send <= 0 ;
                    ws_state <= WSS_BIT_SEND_HIGH ;
                end
            end

            WSS_BIT_SEND_HIGH : begin
                led2812 <= 1'b1 ;

                if (led_data[bit_send]) 
                    if (counter < DELAY_1_HIGH)
                        counter <= counter + 1'd1;
                    else begin
                        counter <= 0 ;
                        ws_state <= WSS_BIT_SEND_LOW ;
                    end
                else 
                    if (counter < DELAY_0_HIGH)
                        counter <= counter + 1'd1 ;
                    else begin
                        counter <= 0 ;
                        ws_state <= WSS_BIT_SEND_LOW ;
                    end
            end

            WSS_BIT_SEND_LOW : begin
                led2812 <= 1'b0 ;

                if (led_data[bit_send]) 
                    if (counter < DELAY_1_LOW) 
                        counter <= counter + 1'd1 ;
                    else begin
                        counter <= 0 ;
                        bit_send <= bit_send + 1'd1 ;
                        ws_state <= WSS_DATA_SEND ;
                    end
                else 
                    if (counter < DELAY_0_LOW) 
                        counter <= counter + 1'd1 ;
                    else begin
                        counter <= 0 ;
                        bit_send <= bit_send + 1'd1 ;
                        ws_state <= WSS_DATA_SEND ;
                end
            end
        endcase
    end
endmodule
