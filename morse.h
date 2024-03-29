static const uint8_t morse[256] PROGMEM = {
	[0x0013] = KEY_SPACE, // ..--
	[0x0005] = KEY_A, // .-
	[0x0018] = KEY_B, // -...
	[0x001a] = KEY_C, // -.-.
	[0x000c] = KEY_D, // -..
	[0x0002] = KEY_E, // .
	[0x0012] = KEY_F, // ..-.
	[0x000e] = KEY_G, // --.
	[0x0010] = KEY_H, // ....
	[0x0004] = KEY_I, // ..
	[0x0017] = KEY_J, // .---
	[0x000d] = KEY_K, // -.-
	[0x0014] = KEY_L, // .-..
	[0x0007] = KEY_M, // --
	[0x0006] = KEY_N, // -.
	[0x000f] = KEY_O, // ---
	[0x0016] = KEY_P, // .--.
	[0x001d] = KEY_Q, // --.-
	[0x000a] = KEY_R, // .-.
	[0x0008] = KEY_S, // ...
	[0x0003] = KEY_T, // -
	[0x0009] = KEY_U, // ..-
	[0x0011] = KEY_V, // ...-
	[0x000b] = KEY_W, // .--
	[0x0019] = KEY_X, // -..-
	[0x001b] = KEY_Y, // -.--
	[0x001c] = KEY_Z, // --..
	[0x002f] = KEY_1, // .----
	[0x0027] = KEY_2, // ..---
	[0x0023] = KEY_3, // ...--
	[0x0021] = KEY_4, // ....-
	[0x0020] = KEY_5, // .....
	[0x0030] = KEY_6, // -....
	[0x0038] = KEY_7, // --...
	[0x003c] = KEY_8, // ---..
	[0x003e] = KEY_9, // ----.
	[0x003f] = KEY_0, // -----
	[0x0055] = KEY_PERIOD, // .-.-.-
	[0x0073] = KEY_COMMA, // --..--
	[0x004c] = KEY_SLASH | 0x80, // ? ..--..
	[0x005e] = KEY_QUOTE, // .----.
	[0x006b] = KEY_1 | 0x80, // ! -.-.--
	[0x0032] = KEY_SLASH, // -..-.
	[0x0036] = KEY_9 | 0x80, // ( -.--.
	[0x006d] = KEY_0 | 0x80, // ) -.--.-
	[0x0028] = KEY_7 | 0x80, // & .-...
	[0x0078] = KEY_SEMICOLON | 0x80, // : ---...
	[0x006a] = KEY_SEMICOLON, // -.-.-.
	[0x0031] = KEY_EQUAL, // -...-
	[0x002a] = KEY_EQUAL | 0x80, // + .-.-.
	[0x0061] = KEY_MINUS, // -....-
	[0x004d] = KEY_MINUS | 0x80, // _ ..--.-
	[0x0052] = KEY_QUOTE | 0x80, // " .-..-.
	[0x0089] = KEY_4 | 0x80, // $ ...-..-
	[0x005a] = KEY_2 | 0x80, // @ .--.-.
};
