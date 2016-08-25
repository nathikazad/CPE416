#pragma once

class BBLCD {
public:
	static void init(void);

	static void set_cursor(uint8_t col, uint8_t row);
	static void clear(void);

	static void print(const char* string);
	static void print(u16 number);
	static void printf(const char *fmt, ...);
	static void scroll_message(const char* message, int length, int iteration);

private:
	static void e_Clk(void);
	static void write_lcd(u08 data);
	static void write_control(u08 data);
	static void write_data(u08 data);

};
