namespace GP22 {

typedef uint32_t T_register;


//Isso define o tamanho de um registro
// Por exemplo. para um registro de 32 bits, use
// typedef uint32_t T_register;
extern T_register registers_data[7];

//Registradores
enum registers
{
    REG0  = 0x00,
    REG1  = 0x01,
    REG2  = 0x02,
    REG3  = 0x03,
    REG4  = 0x04,
    REG5  = 0x05,
    REG6  = 0x06
};

// Uma macro que define valores de codificação para um bitset
// Isso codifica as posições inicial e final de um bitset para um determinado registro em
// os dois primeiros e os dois últimos bytes de um int de 32 bits respectivamente
#define REG_BIT_DEFN(start, end) ((uint32_t(start)<<16)|(end-start+1))


// As configurações
//
// Use a macro `REG_BIT_DEFN` para definir as posições inicial e final de todas as configurações
//disponível no seu dispositivo

enum {
	    REG0_ID0                = REG_BIT_DEFN(0, 7),
	    REG0_NEG_START          = REG_BIT_DEFN(8, 8),
	    REG0_NEG_STOP1          = REG_BIT_DEFN(9, 9),
	    REG0_NEG_STOP2          = REG_BIT_DEFN(10, 10),
	    REG0_MESSB2             = REG_BIT_DEFN(11, 11),
	    REG0_NO_CAL_AUTO        = REG_BIT_DEFN(12, 12),
	    REG0_CALIBRATE          = REG_BIT_DEFN(13, 13),
	    REG0_SEL_ECLK_TMP       = REG_BIT_DEFN(14, 14),
	    REG0_ANZ_FAKE           = REG_BIT_DEFN(15, 15),
	    REG0_TCYCLE             = REG_BIT_DEFN(16, 16),
	    REG0_ANZ_PORT           = REG_BIT_DEFN(17, 17),
	    REG0_START_CLKHS_START  = REG_BIT_DEFN(18, 19),
	    REG0_DIV_CLKHS          = REG_BIT_DEFN(20, 21),
	    REG0_ANZ_PER_CALRES     = REG_BIT_DEFN(22, 23),
	    REG0_DIV_FIRE           = REG_BIT_DEFN(24, 27),
	    REG0_ANZ_FIRE_LSB       = REG_BIT_DEFN(28, 31)
	};

	enum {
	    REG1_ID1                = REG_BIT_DEFN(0, 7),
	    REG1_SEL_TSTO1          = REG_BIT_DEFN(8, 10),
	    REG1_SEL_TSTO2          = REG_BIT_DEFN(11, 13),
	    REG1_SEL_START_FIRE     = REG_BIT_DEFN(14, 14),
	    REG1_CURR32K            = REG_BIT_DEFN(15, 15),
	    REG1_HITIN1             = REG_BIT_DEFN(16, 18),
	    REG1_HITIN2             = REG_BIT_DEFN(19, 21),
	    REG1_KEEP_DEFAULT       = REG_BIT_DEFN(22, 22),
	    REG1_EN_FAST_INIT       = REG_BIT_DEFN(23, 23),
	    REG1_HIT1               = REG_BIT_DEFN(24, 27),
	    REG1_HIT2               = REG_BIT_DEFN(28, 31),
		REG1_TEST_DATA			= REG_BIT_DEFN(24, 31)
	};

	enum {
	    REG2_ID2                = REG_BIT_DEFN(0, 7),
	    REG2_DELVAL1            = REG_BIT_DEFN(8, 26),
	    REG2_RFEDGE1            = REG_BIT_DEFN(27, 27),
	    REG2_RFEDGE2            = REG_BIT_DEFN(28, 28),
	    REG2_EN_INT_ALU         = REG_BIT_DEFN(29, 29),
	    REG2_EN_INT_HITS        = REG_BIT_DEFN(30, 30),
	    REG2_EN_INT_TDC_TIMEOUT = REG_BIT_DEFN(31, 31)
	};

	enum {
	    REG3_ID3                = REG_BIT_DEFN(0, 7),
	    REG3_DELVAL2            = REG_BIT_DEFN(8, 26),
	    REG3_SEL_TIMO_MB2       = REG_BIT_DEFN(27, 28),
	    REG3_EN_ERR_VAL         = REG_BIT_DEFN(29, 29),
	    REG3_EN_FIRST_WAVE      = REG_BIT_DEFN(30, 30),
	    REG3_EN_AUTOCALC_MB2    = REG_BIT_DEFN(31, 31)
	};

	enum {
	    REG4_ID4                = REG_BIT_DEFN(0, 7),
	    REG4_DELVAL3            = REG_BIT_DEFN(8, 26),
	    REG4_KEEP_DEFAULT       = REG_BIT_DEFN(27, 31)
	};


	enum {
	    REG3FW_ID3              = REG_BIT_DEFN(0, 7),
	    REG3FW_DELREL1          = REG_BIT_DEFN(8, 13),
	    REG3FW_DELREL2          = REG_BIT_DEFN(14, 19),
	    REG3FW_DELREL3          = REG_BIT_DEFN(20, 25),
	    REG3FW_KEEP_DEFAULT     = REG_BIT_DEFN(26, 26),
	    REG3FW_SEL_TIMO_MB2     = REG_BIT_DEFN(27, 28),
	    REG3FW_EN_ERR_VAL       = REG_BIT_DEFN(29, 29),
	    REG3FW_EN_FIRST_WAVE    = REG_BIT_DEFN(30, 30),
	    REG3FW_EN_AUTOCALC_MB2  = REG_BIT_DEFN(31, 31)
	};

	enum {
	    REG4FW_ID4              = REG_BIT_DEFN(0, 7),
	    REG4FW_OFFS             = REG_BIT_DEFN(8, 12),
	    REG4FW_OFFSRNG1         = REG_BIT_DEFN(13, 13),
	    REG4FW_OFFSRNG2         = REG_BIT_DEFN(14, 14),
	    REG4FW_EDGE_FW          = REG_BIT_DEFN(15, 15),
	    REG4FW_DIS_PW           = REG_BIT_DEFN(16, 16),
	    REG4FW_KEEP_DEFAULT     = REG_BIT_DEFN(17, 31)
	};

	enum {
	    REG5_ID5                = REG_BIT_DEFN(0, 7),
	    REG5_PHFIRE             = REG_BIT_DEFN(8, 23),
	    REG5_REPEAT_FIRE        = REG_BIT_DEFN(24, 26),
	    REG5_DIS_PHASESHIFT     = REG_BIT_DEFN(27, 27),
	    REG5_EN_STARTNOISE      = REG_BIT_DEFN(28, 28),
	    REG5_CONF_FIRE          = REG_BIT_DEFN(29, 31)
	};

	enum {
	    REG6_ID6                = REG_BIT_DEFN(0, 7),
	    REG6_ANZ_FIRE_MSB       = REG_BIT_DEFN(8, 10),
	    REG6_TEMP_PORTDIR       = REG_BIT_DEFN(11, 11),
	    REG6_DOUBLE_RES         = REG_BIT_DEFN(12, 12),
	    REG6_QUAD_RES           = REG_BIT_DEFN(13, 13),
	    REG6_FIREO_DEF          = REG_BIT_DEFN(14, 14),
	    REG6_HZ60               = REG_BIT_DEFN(15, 15),
	    REG6_CYCLE_TOF          = REG_BIT_DEFN(16, 17),
	    REG6_CYCLE_TEMP         = REG_BIT_DEFN(18, 19),
	    REG6_START_CLKHS_END    = REG_BIT_DEFN(20, 20),
	    REG6_EN_INT_END         = REG_BIT_DEFN(21, 21),
	    REG6_TW2                = REG_BIT_DEFN(22, 23),
	    REG6_EMPTY_0            = REG_BIT_DEFN(24, 24),
	    REG6_DA_KORR            = REG_BIT_DEFN(25, 28),
	    REG6_EMPTY_1            = REG_BIT_DEFN(29, 29),
	    REG6_NEG_STOP_TEMP      = REG_BIT_DEFN(30, 30),
	    REG6_EN_ANALOG          = REG_BIT_DEFN(31, 31)
	};



    inline volatile T_register* regAddress(registers reg)
	{
		return reinterpret_cast<volatile T_register*>((unsigned int)registers_data + reg * sizeof(T_register));
	}

    inline T_register regRead(registers reg)
	{
		return *regAddress(reg);
	}

	inline void regWrite(registers reg, T_register value)
	{
		*regAddress(reg) = value;
	}

    inline T_register bitmaskRead(registers reg, uint32_t bits)
	{
		// Get the current value of the register
		T_register regval = *regAddress(reg);

		// Extract the width of the bits required, stored in the last two bytes
		// of `bits`
		const uint32_t width  = bits & 0xff;

		// Extract the position of the first bit requested, stored in the first
		// two bytes of `bits`
		const uint32_t bitno  = bits >> 16;

		// Shift so that the first interesting bit is in position `1`
		regval >>= bitno;

		// Mask out only the bits below position `width`
		regval  &= (T_register(1<<width)-1);

		// Return
		return regval;
	}
    
	inline void bitmaskWrite(registers reg, uint32_t bits, T_register value)
	{
		// Get the current value of the register
		T_register regval = *regAddress(reg);

		// Extract the width of the bits required, stored in the last two bytes
		// of `bits`
		const uint32_t width  = bits & 0xff;

		// Extract the position of the first bit requested, stored in the first
		// two bytes of `bits`
		const uint32_t bitno  = bits >> 16;

		// Make a positive mask of just this value's position
		const T_register val_mask = T_register((1<<width)-1) << bitno;

		// Set all the bits of this value to zero
		regval &= ~val_mask;

		// Set all the bits of this value to their required new value
		regval |=  (value << bitno) & val_mask;

		// Store
		*regAddress(reg) = regval;
	}

}