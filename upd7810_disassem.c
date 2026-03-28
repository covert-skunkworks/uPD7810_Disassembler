#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

//  gcc upd7810_disassem.c -o upd7810_disassem.exe && ./upd7810_disassem.exe <binary_name>.bin

#define MEMORY_SIZE 65536
#define BANKS 2

#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define RESET   "\033[0m"

#define PRINT(...) { memcpy(&disassem[pc].text[POS_ADDR], raw_str, strlen(raw_str)); snprintf(dis_str, sizeof(dis_str), __VA_ARGS__); memcpy(&disassem[pc].text[POS_DIS], dis_str, strlen(dis_str)); }
#define PRINT_DESC(...) { snprintf(&disassem[pc].text[POS_DESC], POS_MAX-POS_DESC, __VA_ARGS__); }

#define GET_CODE_FLG	mem_flg[cpu.PC] |= 0x10
#define GET_DATA_RD_FLG	mem_flg[cpu.PC] |= 0x0A

enum{ RPA_VA = 0, RPA_BC, RPA_DE, RPA_HL, RPA_DEPP, RPA_HLPP, RPA_DEMM, RPA_HLMM, RPA_MAX };
enum{ R_V = 0, R_A, R_B, R_C, R_D, R_E, R_H, R_L, R_MAX };
enum
{
	SR_PA = 0, SR_PB, SR_PC, SR_PD,
	SR_PF = 5, SR_MKH, SR_MKL, SR_ANM, SR_SMH, SR_SML, SR_EOM, SR_ETMM, SR_TMM,
	SR_MM = 16, SR_MCC, SR_MA, SR_MB, SR_MC,
	SR_MF = 23, SR_TXB, SR_RXB, SR_TM0, SR_TM1,
	SR_CR0 = 32, SR_CR1, SR_CR2, SR_CR3,
	SR_ZCM = 40,
	SR_MAX = 64
};

typedef struct {
	uint8_t regs[BANKS][R_MAX];     // [bank][reg]: 0=V,1=A,2=B,3=C,4=D,5=E,6=H,7=L
	uint16_t EA[BANKS];             // EA
	uint16_t PC;                    // Program counter
	uint16_t SP;                    // Stack pointer
	uint8_t PSW;                    // Program Status Word
	uint8_t bank;                   // Current register bank (0 or 1)
	uint8_t special_regs[SR_MAX];	// Special registers
	uint32_t irq_flg;				// IRQ Flags
	// Peripherals (basic stubs)
	uint8_t MM;                     // Memory mode register
	// Multifunction timer / event counter
	uint16_t ECNT;
	uint16_t ECPT;
	uint16_t ETM0;
	uint16_t ETM1;
	// Interrupt related (basic)
	uint8_t IRR;                    // Interrupt request register
	uint8_t ITE;                    // Interrupt test/enable register
	uint8_t MKL, MKH;               // Interrupt mask low/high
	uint8_t IFF1, IFF2;             // Interrupt flip-flops
} uPD7810;

enum{ POS_ADDR = 0, POS_RAW_BYES = 5, POS_DIS = 18, POS_DESC_COL = 42, POS_DESC = 44, POS_MAX = 128 };
enum{ TYPE_JUMP = 0, TYPE_FUNCTION };
enum{ TABLE_EMPTY = 0, TABLE_FUNCTION, TABLE_DATA, TABLE_MAX };

typedef struct {
	uint8_t state;
	uint8_t table_flg;
	char text[POS_MAX];
} uPD7810_text;

typedef struct {
	uint16_t addr;
	uint16_t count;
	char label[24];
} uPD7810_label_text;

// Flag macros
#define CY (cpu.PSW & 0x01)
#define L0 ((cpu.PSW & 0x04) >> 2)
#define L1 ((cpu.PSW & 0x08) >> 3)
#define HC ((cpu.PSW & 0x10) >> 4)
#define SK ((cpu.PSW & 0x20) >> 5)
#define Z  ((cpu.PSW & 0x40) >> 6)

// Set flag macros
#define SET_CY(val) (cpu.PSW = (cpu.PSW & ~0x01) | ((val) ? 0x01 : 0))
#define SET_L0(val) (cpu.PSW = (cpu.PSW & ~0x04) | ((val) ? 0x04 : 0))
#define SET_L1(val) (cpu.PSW = (cpu.PSW & ~0x08) | ((val) ? 0x08 : 0))
#define SET_HC(val) (cpu.PSW = (cpu.PSW & ~0x10) | ((val) ? 0x10 : 0))
#define SET_SK(val) (cpu.PSW = (cpu.PSW & ~0x20) | ((val) ? 0x20 : 0))
#define SET_Z(val) (cpu.PSW = (cpu.PSW & ~0x40) | ((val) ? 0x40 : 0))

// Register access
#define REG_EA (cpu.EA[cpu.bank])
#define REG_BANK cpu.regs[cpu.bank]
#define REG_V (REG_BANK[R_V])
#define REG_A (REG_BANK[R_A])
#define REG_B (REG_BANK[R_B])
#define REG_C (REG_BANK[R_C])
#define REG_D (REG_BANK[R_D])
#define REG_E (REG_BANK[R_E])
#define REG_H (REG_BANK[R_H])
#define REG_L (REG_BANK[R_L])

// 16-bit register pairs
#define GET_VA (((REG_V) << 8) | (REG_A))
#define GET_BC (((REG_B) << 8) | (REG_C))
#define GET_DE (((REG_D) << 8) | (REG_E))
#define GET_HL (((REG_H) << 8) | (REG_L))

#define SET_VA(x) { REG_V = (x >> 8) & 0xFF; REG_A = x & 0xFF; }
#define SET_BC(x) { REG_B = (x >> 8) & 0xFF; REG_C = x & 0xFF; }
#define SET_DE(x) { REG_D = (x >> 8) & 0xFF; REG_E = x & 0xFF; }
#define SET_HL(x) { REG_H = (x >> 8) & 0xFF; REG_L = x & 0xFF; }

// Function prototypes
void initialize(void);
void load_rom(const char *filename);
int emulate_cycle(int pc);
void handle_interrupts(void);
static uint8_t fetch8(void);
static uint8_t second_byte(void);
static uint8_t third_byte(void);
static uint8_t fourth_byte(void);
static uint8_t read_mem(uint16_t addr);
static uint8_t add8(uint8_t a, uint8_t b, uint8_t c );
static uint16_t add16(uint16_t a, uint16_t b, uint16_t c );
static uint8_t sub8(uint8_t a, uint8_t b, uint8_t c );
static uint16_t sub16(uint16_t a, uint16_t b, uint16_t c );
static uint16_t get_rp(uint8_t rpa);
static void set_rp(uint8_t rpa, uint16_t val);
static uint16_t get_rp3(uint8_t rpa);
static void set_rp3(uint8_t rpa, uint16_t val);
static uint16_t get_rpa(uint8_t rpa);
static uint16_t get_rpa1(uint8_t rpa);
static void set_rpa1(uint8_t rpa, uint16_t val);
static uint16_t get_rpa2(uint8_t rpa, uint8_t byte);
static uint16_t get_rpa3(uint8_t rpa, uint8_t byte);
static void set_r1(uint8_t r1, uint8_t val);
static uint8_t get_r1(uint8_t r1);

// XX -- Not used
const char *str_rp_names[4] = {"SP", "BC", "DE", "HL"};
const char *str_rp3_names[4] = {"XX", "BC", "DE", "HL"};
const char *str_rpa_names[16] = {
	"XX", "BC", "DE", "HL", "DE+", "HL+", "DE-", "HL-",
	"XX", "XX", "XX", "DE+byte", "HL+A", "HL+B", "HL+EA", "HL+byte"};
const char *str_rpa1_names[5] = { "VA", "BC", "DE", "HL", "EA" };
const char *str_rpa2_names[16] = {
	"XX", "BC", "DE", "HL", "DE+", "HL+", "DE-", "HL-",
	"XX", "XX", "XX", "DE+byte", "HL+A", "HL+B", "HL+EA", "HL+byte"};
// LDEAX rpa3, STEAX rpa3
const char *str_rpa3_names[16] = {
	"XX", "XX", "DE", "HL", "DE++", "HL++", "DE--", "HL--",
	"XX", "XX", "XX", "DE+byte", "HL+A", "HL+B", "HL+EA", "HL+byte"};
const char *str_r_names[8] = { "V", "A", "B", "C", "D", "E", "H", "L" };
const char *str_r1_names[8] = { "EAH", "EAL", "B", "C", "D", "E", "H", "L" };
const char *str_sr_names[64] = { 
	"PA",  "PB",  "PC",  "PD",  "XX",   "PF",  "MKH", "MKL",
	"ANM", "SMH", "SML", "EOM", "ETMM", "TMM", "XX",  "XX",
	"MM",  "MCC", "MA",  "MB",  "MC",   "XX",  "XX",  "MF",
	"TXB", "RXB", "TM0", "TM1", "XX",   "XX",  "XX",  "XX",
	"CR0", "CR1", "CR2", "CR3", "XX",   "XX",  "XX",  "XX",
	"ZCM", "XX",  "XX",  "XX",  "XX",   "XX",  "XX",  "XX",
	"XX",  "XX",  "XX",  "XX",  "XX",   "XX",  "XX",  "XX",
	"XX",  "XX",  "XX",  "XX",  "XX",   "XX",  "XX",  "XX"
};
const char *str_irf_names[32] = {
	"INTFNMI", "INTFT0", "INTFT1", "INTF1", "INTF2", "INTFE0", "INTFE1", "INTFEIN",
	"INTFAD", "INTFSR", "INTFST", "ER", "OV", "XX", "XX", "XX",
	"AN4", "AN5", "AN6", "AN7", "SB", "XX", "XX", "XX",
	"XX", "XX", "XX", "XX", "XX", "XX", "XX", "XX"
};
const uint8_t default_special_regs[SR_MAX] ={
	[SR_MM]  = 0xF0,
	[SR_MKL] = 0xFF,
	[SR_MKH] = 0xFF,
	[SR_SMH] = 0x00,	// Serial port settings
	[SR_ANM] = 0xE0
};

//							0		4		8				10			18				20			28			60
const char *irq_name[] = {"Reset", "NMI", "Interval_Timer", "External", "Multi_Timer", "ADC",       "Serial",  "Softi" };
const char irq_addr[] = { 0x00, 0x04, 0x08, 0x10, 0x18, 0x20, 0x28, 0x60 };
uint8_t irq_cnt = 0;

uint8_t mem_flg[MEMORY_SIZE];
uint8_t rom[MEMORY_SIZE];
size_t rom_size = 0;

uint8_t rom_flg[MEMORY_SIZE];
FILE *f;
const char *input_file = "in.bin";
const char *output_file = NULL;
long bin_offset = 0;

char raw_str[17];
char dis_str[64];
char desc_str[64];
char *desc = NULL;

uint8_t condition_set;
uint8_t table_instruction;
uint8_t softi_used;

uPD7810 cpu;

uPD7810_text disassem[MEMORY_SIZE];
uPD7810_label_text labels[MEMORY_SIZE];
uint16_t labels_total_cnt;
uint16_t last_label_cnt;
uint16_t last_func_cnt;

uPD7810 registers_bkp[MEMORY_SIZE];
uint16_t registers_bkp_cnt;

void set_sr(uint8_t x, uint8_t val) { cpu.special_regs[x] = val; }
uint8_t get_sr(uint8_t x) { return cpu.special_regs[x]; }

void init_disassemble_buffer(void)
{
	char str[16];

	// Clear buffers
	memset(&disassem, ' ', sizeof(disassem));
	memset(&labels, ' ', sizeof(labels));
	for(int i = 0; i < (sizeof(disassem) / sizeof(disassem[0])); i++ )
	{
		disassem[i].text[POS_MAX - 1] = 0;
		disassem[i].text[POS_DESC_COL] = ';';
		disassem[i].text[POS_DESC] = 0;
		disassem[i].text[0] = 0;
		disassem[i].state = 0;
		disassem[i].table_flg = TABLE_EMPTY;
		labels[i].addr = 0;
		labels[i].count = 0;
	}

	labels_total_cnt = 0;

	// Fill interrupt vector strings
	for(int i = 0; i < (sizeof(irq_addr) / sizeof(irq_addr[0])); i++ )
	{
		snprintf(labels[labels_total_cnt].label, sizeof(labels[labels_total_cnt].label), "IRQ%u_%s\0", i, irq_name[i] );
		labels[labels_total_cnt].addr = irq_addr[i] + bin_offset;
		labels[labels_total_cnt].count++;
		labels_total_cnt++;
	}

	last_label_cnt = 1;
	last_func_cnt = 1;

	memset(&registers_bkp, 0, sizeof(registers_bkp));
	registers_bkp_cnt = 0;
}

void push_registers(uint16_t pc)
{
	registers_bkp[registers_bkp_cnt] = cpu;
	registers_bkp[registers_bkp_cnt].PC = pc;
	registers_bkp_cnt++;

	if(registers_bkp_cnt > (sizeof(registers_bkp) / sizeof(registers_bkp[0]))) {
		printf("\r\nError: No more space left in the table!\r\n");
		exit(1);
	}
}

void pop_registers(void)
{
	if(registers_bkp_cnt) {
		registers_bkp_cnt--;
		cpu = registers_bkp[registers_bkp_cnt];
	} else {
		printf("\r\nError: No registers stored!\r\n");
	}
}

int find_label(uint16_t addr)
{
	int found = 0;

	// Check if label already exists
	for(int i = 0; i < labels_total_cnt; i++ )
	{
		if(addr == labels[i].addr) { found = i + 1; break; }
	}

	return found;
}

void add_label(uint16_t pc, uint16_t addr, uint8_t type)
{
	char str[16];
	int found = 0;

	// Check if label already exists
	for(int i = 0; i < labels_total_cnt; i++ )
	{
		if(addr == labels[i].addr) { labels[i].count++; found = 1; break; }
	}

	if( 0 == found )
	{
		found = 0;

		// Find if address is an IRQ vector
		for(int i = 0; i < (sizeof(irq_addr) / sizeof(irq_addr[0])); i++ )
		{
			if(pc == irq_addr[i]) { found = i + 1; break; }
		}

		if( found )
		{
			snprintf(labels[labels_total_cnt].label, sizeof(labels[labels_total_cnt].label), "%s\0", irq_name[found - 1] );
		}
		else
		{
			if(type) {
				snprintf(labels[labels_total_cnt].label, sizeof(labels[labels_total_cnt].label), "func_%u\0", last_func_cnt );
				last_func_cnt++;
			} else {
				snprintf(labels[labels_total_cnt].label, sizeof(labels[labels_total_cnt].label), "label_%u\0", last_label_cnt );
				last_label_cnt++;
			}
		}

		labels[labels_total_cnt].addr = addr;
		labels[labels_total_cnt].count = 1;
		labels_total_cnt++;
	}
}

int process_table(uint32_t i){
	uint16_t tmp16;
	int diff = 0;

	// Find next used location
	diff = 0;
	for(int j = i; j < rom_size; j++ )
	{
		if(disassem[j].state) { diff = j - i; break; }
	}

	if( (diff > 255) || (diff < 1) ) { diff = 255; }

	for(int k = 0; k < diff; k += 2)
	{
		uint16_t pc = i + k;
		tmp16 = (rom[pc + 1] << 8) | rom[pc];
		snprintf(&raw_str[0], sizeof(raw_str), "%04X %02x %02x      ", i + k, rom[pc], rom[pc + 1]);
		PRINT(".dw %04Xh", tmp16);
		disassem[pc].state = 1;
		disassem[pc + 1].state = 8;
	}

	disassem[i].table_flg = TABLE_EMPTY;
	return diff;
}

int process_jb_table(uint32_t i){
	uint16_t pc = i;
	uint16_t tmp16 = (rom[pc + 1] << 8) | rom[pc];
	snprintf(&raw_str[0], sizeof(raw_str), "%04X %02x %02x      ", pc, rom[pc], rom[pc + 1]);
	PRINT(".dw %04Xh", tmp16);
	disassem[pc].state = 1;
	disassem[pc + 1].state = 8;
	push_registers(tmp16);
	add_label(pc, tmp16, TYPE_JUMP);

	disassem[pc].table_flg = TABLE_EMPTY;
	pc += 2;
	if( !disassem[pc].state ){
		disassem[pc].table_flg = TABLE_FUNCTION;
	}
	return 3;
}

void usage(const char *progname) {
    fprintf(stderr, "Usage: %s [-i input_file] [-o output_file] [-s offset] [-h]\n", progname);
    fprintf(stderr, "Defaults:\n");
    fprintf(stderr, "  -i: %s\n", input_file);
    fprintf(stderr, "  -o: <input file>.asm\n");
    fprintf(stderr, "  -s: %d (bytes from start of input; supports hex with 0x prefix, e.g., 0x100)\n", bin_offset);
    fprintf(stderr, "Description: Copies input file to output, starting from the specified offset.\n");
    exit(EXIT_FAILURE);
}

int main(int argc, char *argv[]) {
	int ret, table_process_loop_flg;
	char fname[256];
	condition_set = 0;
	table_instruction = 0;
	softi_used = 0;

	printf("uPD78C1x Disassembler v1.0\r\n\r\n");

	// Manual argument parsing
	for (int i = 1; i < argc; i++) {
		if (strcmp(argv[i], "-i") == 0) {
			if (++i < argc) {
				input_file = argv[i];
			} else {
				fprintf(stderr, "Error: -i requires an argument.\n");
				usage(argv[0]);
			}
		} else if (strcmp(argv[i], "-o") == 0) {
			if (++i < argc) {
				output_file = argv[i];
			} else {
				fprintf(stderr, "Error: -o requires an argument.\n");
				usage(argv[0]);
			}
		} else if (strcmp(argv[i], "-s") == 0) {
			if (++i < argc) {
				char *endptr;
				bin_offset = strtol(argv[i], &endptr, 0);
				if (*endptr != '\0' || bin_offset < 0) {
					fprintf(stderr, "Error: Invalid offset '%s'. Must be a non-negative integer.\n", argv[i]);
					usage(argv[0]);
				}
			} else {
				fprintf(stderr, "Error: -s requires an argument.\n");
				usage(argv[0]);
			}
		} else if (strcmp(argv[i], "-h") == 0) {
			usage(argv[0]);
		} else {
			fprintf(stderr, "Error: Unknown option '%s'.\n", argv[i]);
			usage(argv[0]);
		}
	}

	init_disassemble_buffer();
	memset(&rom_flg, 0, sizeof(rom_flg));

	if(NULL == output_file) {
		snprintf(fname, sizeof(fname), "%s.asm", input_file);
	} else {
		snprintf(fname, sizeof(fname), "%s", output_file);
	}

	f = fopen(fname, "wb");
	if ( !f ) {
		printf("Can not open log file!\n");
		return 2;
	}

	load_rom(input_file);

	printf("   Offset 0x%04x\r\n", bin_offset);
	printf("   Loaded %u bytes!\r\n", rom_size);

	initialize();

	printf("   First pass...\r\n");
	for (int h = 0; h < ((sizeof(irq_addr) / sizeof(irq_addr[0])) - (1 - softi_used)); ++h)
	{
		cpu.PC = irq_addr[irq_cnt] + bin_offset;
		printf("\tIRQ: %s\r\n", irq_name[irq_cnt]);
		for (int i = bin_offset; i < (rom_size + bin_offset); ++i)
		{
			memset(&raw_str[0], ' ', sizeof(raw_str));
			cpu.PC = emulate_cycle(cpu.PC);

			check_again:
			if( disassem[cpu.PC].state ) {
				// Instruction already processed
				if(registers_bkp_cnt) {
					pop_registers();
					goto check_again;
				} else {
					i = (rom_size + bin_offset);
				}
			}
		}
		irq_cnt++;
	}

	// Process TABLE entries
	printf("   Second TABLE JB process pass");
	do {
		printf(".");
		table_process_loop_flg = 0;
		for (int i = bin_offset; i < (rom_size + bin_offset); ++i)
		{
			if( (TABLE_EMPTY < disassem[i].table_flg) &&
				(TABLE_DATA  > disassem[i].table_flg) ) {
				i += process_jb_table(i);
				table_process_loop_flg = 1;
			}
		}

		// If there are functions in the jump tables, process them
		if( registers_bkp_cnt )
		{
			pop_registers();
			for (int i = bin_offset; i < (rom_size + bin_offset); ++i)
			{
				if( disassem[cpu.PC].state ) {
					// Instruction already processed
					if(registers_bkp_cnt) {
						pop_registers();
					} else {
						i = (rom_size + bin_offset);
					}
				} else {
					memset(&raw_str[0], ' ', sizeof(raw_str));
					cpu.PC = emulate_cycle(cpu.PC);
				}
			}
		}
	} while(table_process_loop_flg);
	printf("\r\n");

	printf("    Third TABLE JR process pass");
	table_process_loop_flg = 1;
	while(table_process_loop_flg){
		printf(".");
		table_process_loop_flg = 0;
		for (int i = bin_offset; i < (rom_size + bin_offset); ++i)
		{
			if( TABLE_DATA == disassem[i].table_flg ) {
				i += process_table(i);
				table_process_loop_flg = 1;
			}
		}

		// If there are functions in the jump tables, process them
		if( registers_bkp_cnt )
		{
			pop_registers();
			for (int i = bin_offset; i < (rom_size + bin_offset); ++i)
			{
				if( disassem[cpu.PC].state ) {
					// Instruction already processed
					if(registers_bkp_cnt) {
						pop_registers();
					} else {
						i = (rom_size + bin_offset);
					}
				} else {
					memset(&raw_str[0], ' ', sizeof(raw_str));
					cpu.PC = emulate_cycle(cpu.PC);
				}
			}
		}
	}
	printf("\r\n");

	printf("     Saving data...\r\n");
	// Print result to the output file
	for (int i = bin_offset; i < (rom_size + bin_offset); ++i)
	{
		int found = find_label(i);

		// If SOFTI instruction not used, do not print vector label
		if(((0x60 + bin_offset) == i) && !softi_used ) { found = 0; }

		// If label found print it
		if(found) { fprintf(f, "\r\n%s:\t\t; %u calls\r\n", labels[found - 1].label, labels[found - 1].count); }

		if( (1 == disassem[i].state) && (' ' != disassem[i].text[0]) ) {
			fprintf(f, "%s\r\n", disassem[i].text);
		} else if( (0 == disassem[i].state) ) {
			fprintf(f, "%04x %02x           .db %02Xh\r\n", i, rom[i] & 0xFF, rom[i] & 0xFF);
		}
	}

	fclose(f);

	printf("Process finished!\r\n\r\n");

	return 0;
}

void initialize(void) {
	memset(&cpu, 0, sizeof(uPD7810));
	memset(&mem_flg[0], ' ', MEMORY_SIZE);
	cpu.PC = 0x0000;
	cpu.SP = 0xFFFE;
	cpu.bank = 0;
	// Initialize peripherals to default
	cpu.MM = 0xFF; // All internal RAM
	cpu.IRR = 0;
	cpu.ITE = 0;
	cpu.MKL = 0xFF; // Mask all
	cpu.MKH = 0xFF;
	cpu.IFF1 = 0;
	cpu.IFF2 = 0;
}

void load_rom(const char *filename)
{
	FILE *from = fopen(filename, "rb");
	if (from == NULL) {
		fprintf(stderr, "Failed to open ROM: %s\n", filename);
		exit(1);
	}
	rom_size = fread(&rom[bin_offset], 1, sizeof(rom), from);
	fclose(from);
}

int emulate_cycle(int pc) {
	uint8_t op, op2 = 0, tmp8 = 0, tmp8_1 = 0, src = 0, offset = 0;
	uint16_t tmp16 = 0, addr = 0, loc_pc = 0;
	uint32_t tmp32 = 0;

	cpu.PC = pc;

	if(condition_set) { condition_set++; }
	if(table_instruction) { table_instruction++; }

	GET_CODE_FLG;
	op = fetch8();
	rom_flg[cpu.PC] |= 1;
	snprintf(&raw_str[0], sizeof(raw_str), "%04x %02x         ", cpu.PC - 1, op);

	switch (op) {
		case 0x72:		// SOFTI
			PRINT("SOFTI");
			PRINT_DESC("Software interrupt call");
			softi_used = 1;
		break;

		case 0x65:		// NEIW wa, byte
			condition_set = 1;
			offset = second_byte();
			tmp8 = third_byte();
			addr = (REG_BANK[R_V] << 8) | offset;
			PRINT("NEIW 0x%02X, 0x%02X", offset, tmp8);
			PRINT_DESC("if(@0x%04X != 0x%02X) {skip next}", addr, tmp8);
		break;

		case 0x76:		// SBI A, byte
			tmp8 = second_byte();
			PRINT("SBI A, 0x%02X", tmp8);
			PRINT_DESC("A -= (0x%02X + CY);", tmp8);
		break;

		case 0x66:		// SUI A, byte
			tmp8 = second_byte();
			PRINT("SUI A, 0x%02X", tmp8);
			PRINT_DESC("A -= 0x%02X;", tmp8);
		break;

		case 0x36:		// SUINB A, byte
			condition_set = 1;
			tmp8 = second_byte();
			PRINT("SUINB A, 0x%02X", tmp8);
			PRINT_DESC("A -= 0x%02X; if(A < 0) {skip next}", tmp8);
		break;
		case 0x21:		// JB 	Jump BC indirect
			PRINT("JB");
			PRINT_DESC("cpu.PC = BC; // Jump to address stored in BC");
			if(table_instruction)
			{
				table_instruction = 0;
				disassem[cpu.PC].table_flg = TABLE_FUNCTION;
				cpu.PC--;
			} else
			{
				cpu.PC = GET_BC;
			}
		break;

		case 0xA8:		// INX EA
			PRINT("INX EA");
			PRINT_DESC("EA++;", REG_EA);
			REG_EA = REG_EA + 1;
		break;

		case 0x02:		// INX rp
		case 0x12:
		case 0x22:
		case 0x32:
			src = (op >> 4) & 3;
			tmp16 = get_rp(src) + 1;
			set_rp(src, tmp16);
			PRINT("INX @%s", str_rp_names[src]);
			PRINT_DESC("%s++;", str_rp_names[src]);
		break;

		case 0x54:		// JMP word
			tmp8 = second_byte();
			tmp8_1 = third_byte();
			loc_pc = cpu.PC;
			addr = (tmp8_1 << 8) | tmp8;
			cpu.PC = addr;
			add_label(loc_pc - 3, cpu.PC, TYPE_JUMP);

			tmp32 = find_label(addr);
			if(tmp32){
				PRINT("JMP %s", labels[tmp32 - 1].label);
				PRINT_DESC("goto %s;", labels[tmp32 - 1].label);
			} else {
				PRINT("JMP @0x%04X", cpu.PC);
				PRINT_DESC("goto 0x%04X;", cpu.PC);
			}

			if(condition_set)
			{
				condition_set = 0;
				push_registers(cpu.PC);
				cpu.PC = loc_pc;
			}
		break;
		case 0xA0:		// POP rp1
		case 0xA1:
		case 0xA2:
		case 0xA3:
		case 0xA4:
			tmp8 = op & 7;
			PRINT("POP %s", str_rpa1_names[tmp8]);
			PRINT_DESC("Pull from stack and load to %s", str_rpa1_names[tmp8]);
		break;

		case 0x20:		// INRW wa
			condition_set = 1;
			offset = second_byte();
			addr = (REG_BANK[R_V] << 8) | offset;
			tmp8 = read_mem(addr);
			PRINT("INRW 0x%02X", offset);
			PRINT_DESC("@0x%04X += 1; (if carry, skip next)", addr);
			tmp8 = add8(tmp8, 1, 0);
		break;

		case 0x63:		// STAW wa
			offset = second_byte();
			addr = (REG_BANK[R_V] << 8) | offset;
			PRINT("STAW 0x%02X", offset);
			PRINT_DESC("@0x%04X = A;", addr);
		break;

		case 0x01:		// LDAW wa
			offset = second_byte();
			addr = (REG_BANK[R_V] << 8) | offset;
			REG_A = read_mem(addr);
			PRINT("LDAW 0x%02X", offset);
			PRINT_DESC("A = @0x%04X;", addr);
			SET_L0(0); SET_L1(0); SET_SK(0);
		break;

		case 0x27:		// GTI A, byte
			condition_set = 1;
			tmp8 = second_byte();
			PRINT("GTI A, 0x%02X", tmp8);
			PRINT_DESC("if(A > 0x%02X) {skip next}", tmp8);
		break;

		case 0x37:		// LTI A, byte
			condition_set = 1;
			tmp8 = second_byte();
			PRINT("LTI A, 0x%02X", tmp8);
			PRINT_DESC("if(A < 0x%02X) {skip next}", tmp8);
		break;

		case 0xB0:		// PUSH rp1
		case 0xB1:
		case 0xB2:
		case 0xB3:
		case 0xB4:
			tmp8 = op & 7;
			tmp16 = get_rpa1(tmp8);
			PRINT("PUSH %s", str_rpa1_names[tmp8]);
			PRINT_DESC("Push %s to stack", str_rpa1_names[tmp8]);
		break;

		case 0x41:		// INR r2
		case 0x42:
		case 0x43:
			condition_set = 1;
			REG_BANK[op & 3] = add8(REG_BANK[op & 3], 1, 0);
			PRINT("INR %s", str_r_names[op & 3]);
			PRINT_DESC("%s++; if(%s > 255) {skip next}", str_r_names[op & 3], str_r_names[op & 3]);
		break;

		case 0x47:		// ONI A, byte
			condition_set = 1;
			tmp8 = second_byte();
			PRINT("ONI A, 0x%02X", tmp8);
			PRINT_DESC("if( A & 0x%02X ) {skip next}", tmp8);
		break;

		case 0x57:		// OFFI A, byte
			condition_set = 1;
			tmp8 = second_byte();
			PRINT("OFFI A, 0x%02X", tmp8);
			PRINT_DESC("if( 0 == (A & 0x%02X) ) {skip next}", tmp8);
		break;

		case 0x29:		// LDAX rpa2
		case 0x2A:
		case 0x2B:
		case 0x2C:
		case 0x2D:
		case 0x2E:
		case 0x2F:
		case 0xAC:
		case 0xAD:
		case 0xAE:
			src = ((op >> 4) & 8) | (op & 7);
			addr = get_rpa2(src, 0);
			REG_A = read_mem(addr);
			PRINT("LDAX {%s}", str_rpa2_names[src]);
			PRINT_DESC("A = @(0x%04X);", addr);
		break;
		case 0xAB:
		case 0xAF:
			tmp8 = second_byte();
			src = ((op >> 4) & 8) | (op & 7);
			addr = get_rpa2(src, tmp8);
			REG_A = read_mem(addr);
			PRINT("LDAX {%s}", str_rpa2_names[src]);
			PRINT_DESC("A = @{0x%04X);", addr);
		break;

		case 0x45:		// ONIW wa, byte
			condition_set = 1;
			offset = second_byte();
			tmp8 = third_byte();
			addr = (REG_BANK[R_V] << 8) | offset;
			tmp8_1 = read_mem(addr);
			PRINT("ONIW 0x%02X, 0x%02X", offset, tmp8);
			PRINT_DESC("if(@0x%04X & 0x%02X) {skip next}", addr, tmp8);
		break;

		case 0x55:		// OFFIW wa, byte
			condition_set = 1;
			offset = second_byte();
			tmp8 = third_byte();
			addr = (REG_BANK[R_V] << 8) | offset;
			PRINT("OFFIW 0x%02X, 0x%02X", offset, tmp8);
			PRINT_DESC("if( 0 == (@0x%04X & 0x%02X)) {skip next}", addr, tmp8);
		break;

		case 0x71:		// MVIW wa, byte
			offset = second_byte();
			tmp8 = third_byte();
			addr = (REG_BANK[R_V] << 8) | offset;
			PRINT("MVIW 0x%02X, 0x%02X", offset, tmp8);
			PRINT_DESC("@0x%04X = 0x%02X;", addr, tmp8);
		break;

		case 0x62:		// RETI
			cpu.PC--;
			PRINT("RETI");
			PRINT_DESC("// Return from interrupt");
		break;

		case 0xB8:		// RET
			if(condition_set)
			{
				condition_set = 0;
				push_registers(cpu.PC);
			} else {
				cpu.PC--;
			}
			PRINT("RET");
			PRINT_DESC("// Return from subroutine");
		break;

		case 0xB9:		// RETS
			if(condition_set)
			{
				condition_set = 0;
				push_registers(cpu.PC);
			} else {
				cpu.PC--;
			}
			PRINT("RETS 0x%04X", cpu.PC);
			PRINT_DESC("// Return from subroutine and skip next instruction");
		break;

		case 0x49:		// MVIX rpa1, byte
		case 0x4A:
		case 0x4B:
			src = op & 3;
			tmp8 = second_byte();
			addr = get_rpa(src);
			PRINT("MVIX {%s}, 0x%02X", str_rp_names[src], tmp8);
			PRINT_DESC("@0x%04X = 0x%02X;", addr, tmp8);
		break;

		case 0x15:		// ORIW wa, byte
			offset = second_byte();
			tmp8_1 = third_byte();
			addr = (REG_BANK[R_V] << 8) | offset;
			PRINT("ORIW 0x%02X, 0x%02X", offset, tmp8_1);
			PRINT_DESC("@0x%04X |= 0x%02X;", addr, tmp8_1);
		break;
		case 0x17:		// ORI A, byte
			tmp8 = second_byte();
			PRINT("ORI A, 0x%02X", tmp8);
			PRINT_DESC("A |= 0x%02X;", tmp8);
			REG_A |= tmp8;
		break;

		case 0xBC:		// STAX rpa2
		case 0xBD:
		case 0xBE:
		case 0x39:
		case 0x3A:
		case 0x3B:
		case 0x3C:
		case 0x3D:
		case 0x3E:
		case 0x3F:
			src = ((op >> 4) & 8) | (op & 7);
			addr = get_rpa2(src, 0);
			PRINT("STAX {%s}, A", str_rpa2_names[src]);
			PRINT_DESC("@0x%04X = A;", addr);
		break;
		case 0xBB:
		case 0xBF:
			tmp8 = second_byte();
			src = ((op >> 4) & 8) | (op & 7);
			addr = get_rpa2(src, tmp8);
			PRINT("STAX {%s + 0x%02X}, A", str_rpa2_names[src], tmp8);
			PRINT_DESC("@{%s + 0x%02X} = A;", str_rpa2_names[src], tmp8);
		break;

		case 0x68:		// MVI r, byte
		case 0x69:
		case 0x6A:
		case 0x6B:
		case 0x6C:
		case 0x6D:
		case 0x6E:
		case 0x6F:
			tmp8 = second_byte();
			PRINT("MVI %s, 0x%02X", str_r_names[op & 7], tmp8);
			PRINT_DESC("%s = 0x%02X;", str_r_names[op & 7], tmp8);
			REG_BANK[op & 7] = tmp8;
		break;

		case 0x04:		// LXI rp2, word
		case 0x14:
		case 0x24:
		case 0x34:
		case 0x44:
			src = (op >> 4) & 7;
			tmp8 = second_byte();
			tmp8_1 = third_byte();
			tmp16 = (tmp8_1 << 8) | tmp8;
			switch( src ){
				case 0:
					cpu.SP = tmp16;
					desc = "SP";
				break;
				case 1:
					SET_BC(tmp16);
					desc = "BC";
				break;
				case 2:
					SET_DE(tmp16);
					desc = "DE";
				break;
				case 3:
					SET_HL(tmp16);
					desc = "HL";
				break;
				case 4:
					desc = "EA";
				break;
				default:
					desc = "rp2, word ERROR!!! ";
					printf("\r\n%sUnknown rp2: 0x%02X at PC=0x%04X%s\n", RED, src, cpu.PC - 1, RESET);
				break;
			}
			PRINT("LXI %s, 0x%04X", desc, tmp16);
			PRINT_DESC("%s = 0x%04X;", desc, tmp16)
		break;

		case 0x4C:
			GET_CODE_FLG;
			op2 = second_byte();
			switch( op2 )
			{
				case 0xC0:		// MOV A, sr1
				case 0xC1:
				case 0xC2:
				case 0xC3:
				case 0xC5:
				case 0xC6:
				case 0xC7:
				case 0xC8:
				case 0xC9:
				case 0xCB:
				case 0xCD:
				case 0xD9:
				case 0xE0:
				case 0xE1:
				case 0xE2:
				case 0xE3:
					tmp8 = op2 & 0x3F;
					PRINT("MOV A, %s", str_sr_names[tmp8]);
					PRINT_DESC("A = %s;", str_sr_names[tmp8]);
				break;/*
				case 0x40:
					tmp8 = op2 & 0x3F;
					PRINT("UNKNOWN");
					PRINT_DESC("// Used in casio CZ101;");
				break;*/
				default:
					printf("\r\n%sUnknown opcode: 0x%02X :: 0x%02X at PC=0x%04X%s\n", RED, op, op2, cpu.PC - 1, RESET);
				break;
			}
		break;

		case 0x4D:
			GET_CODE_FLG;
			op2 = second_byte();
			switch( op2 )
			{
				case 0xC0:		// MOV sr, A
				case 0xC1:
				case 0xC2:
				case 0xC3:
				case 0xC5:
				case 0xC6:
				case 0xC7:
				case 0xC8:
				case 0xC9:
				case 0xCA:
				case 0xCB:
				case 0xCC:
				case 0xCD:
				case 0xD0:
				case 0xD1:
				case 0xD2:
				case 0xD3:
				case 0xD4:
				case 0xD7:
				case 0xD8:
				case 0xDA:
				case 0xDB:
				case 0xE8:
					tmp8 = op2 & 0x3F;
					PRINT("MOV %s, A", str_sr_names[tmp8]);
					PRINT_DESC("%s = A;", str_sr_names[tmp8]);
					set_sr(tmp8, REG_A);
				break;
				default:
					printf("\r\n%sUnknown opcode: 0x%02X :: 0x%02X at PC=0x%04X%s\n", RED, op, op2, cpu.PC - 1, RESET);
				break;
			}
		break;

		case 0x4E:		// JRE word
		case 0x4F:
			tmp8 = second_byte();
			loc_pc = cpu.PC;
			if(op & 1){
				cpu.PC -= (~(tmp8 - 1) & 0xFF);
			} else {
				cpu.PC += tmp8;
			}

			add_label(loc_pc - 2, cpu.PC, TYPE_JUMP);

			tmp32 = find_label(cpu.PC);
			if(tmp32){
				PRINT("JRE %s", labels[tmp32 - 1].label);
				PRINT_DESC("goto %s;", labels[tmp32 - 1].label);
			} else {
				PRINT("JRE @0x%04X", cpu.PC);
				PRINT_DESC("goto 0x%04X;", cpu.PC);
			}

			if(condition_set)
			{
				condition_set = 0;
				push_registers(cpu.PC);
				cpu.PC = loc_pc;
			}
		break;

		case 0x05:		// ANIW wa, byte
			offset = second_byte();
			addr = (REG_BANK[R_V] << 8) | offset;
			tmp8 = read_mem(addr);
			tmp8_1 = third_byte();
			PRINT("ANIW 0x%02X, 0x%02X", offset, tmp8_1);
			PRINT_DESC("@0x%04X &= 0x%02X;", addr, tmp8_1);
		break;

		case 0x07:		// ANI A, byte
			tmp8 = second_byte();
			PRINT("ANI A, 0x%02X", tmp8);
			PRINT_DESC("A &= 0x%02X;", tmp8);
		break;

		case 0x10:		// EXA
			tmp8 = cpu.regs[0][R_V];
			cpu.regs[0][R_V] = cpu.regs[1][R_V];
			cpu.regs[1][R_V] = tmp8;

			tmp8 = cpu.regs[0][R_A];
			cpu.regs[0][R_A] = cpu.regs[1][R_A];
			cpu.regs[1][R_A] = tmp8;

			tmp16 = cpu.EA[0];
			cpu.EA[0] = cpu.EA[1];
			cpu.EA[1] = tmp16;
			PRINT("EXA");
			PRINT_DESC("A <-> A' V <-> V' EA <-> EA'");
		break;

		case 0x11:		// EXX
			tmp8 = cpu.regs[0][R_B];
			cpu.regs[0][R_B] = cpu.regs[1][R_B];
			cpu.regs[1][R_B] = tmp8;

			tmp8 = cpu.regs[0][R_C];
			cpu.regs[0][R_C] = cpu.regs[1][R_C];
			cpu.regs[1][R_C] = tmp8;

			tmp8 = cpu.regs[0][R_D];
			cpu.regs[0][R_D] = cpu.regs[1][R_D];
			cpu.regs[1][R_D] = tmp8;

			tmp8 = cpu.regs[0][R_E];
			cpu.regs[0][R_E] = cpu.regs[1][R_E];
			cpu.regs[1][R_E] = tmp8;

			tmp8 = cpu.regs[0][R_H];
			cpu.regs[0][R_H] = cpu.regs[1][R_H];
			cpu.regs[1][R_H] = tmp8;

			tmp8 = cpu.regs[0][R_L];
			cpu.regs[0][R_L] = cpu.regs[1][R_L];
			cpu.regs[1][R_L] = tmp8;

			PRINT("EXX");
			PRINT_DESC("BC <-> BC' DE <-> DE' HL <-> HL'");
		break;

		case 0x16:		// XRI A, byte
			tmp8 = second_byte();
			PRINT("XRI A, 0x%02X", tmp8);
			PRINT_DESC("A ^= 0x%02X;", tmp8);
			REG_A ^= tmp8;
		break;

		case 0x50:		// EXH
			tmp8 = cpu.regs[0][R_H];
			cpu.regs[0][R_H] = cpu.regs[1][R_H];
			cpu.regs[1][R_H] = tmp8;

			tmp8 = cpu.regs[0][R_L];
			cpu.regs[0][R_L] = cpu.regs[1][R_L];
			cpu.regs[1][R_L] = tmp8;

			PRINT("EXH");
			PRINT_DESC("HL <-> HL'");
		break;

		case 0x67:		// NEI A, byte
			condition_set = 1;
			tmp8 = second_byte();
			PRINT("NEI A, 0x%02X", tmp8);
			PRINT_DESC("if(A != 0x%02X) {skip next}", tmp8);
		break;

		case 0x77:		// EQI A, byte
			condition_set = 1;
			tmp8 = second_byte();
			PRINT("EQI A, 0x%02X", tmp8);
			PRINT_DESC("if(A == 0x%02X) {skip next}", tmp8);
		break;

		case 0xAA:		// EI
			PRINT("EI");
			PRINT_DESC("Enable Interrupts")
		break;

		case 0xA5:		// DMOV EA, rp3
		case 0xA6:
		case 0xA7:
			tmp16 = get_rp3((op & 3));
			PRINT("DMOV EA, %s", str_rp3_names[op & 3]);
			PRINT_DESC("EA = %s;", str_rp3_names[op & 3]);
			REG_EA = tmp16;
		break;

		case 0xBA:		// DI
			PRINT("DI");
			PRINT_DESC("Disable Interrupts")
		break;

		case 0xB5:		// DMOV rp3, EA
		case 0xB6:
		case 0xB7:
			set_rp3((op & 3), REG_EA);
			PRINT("DMOV %s, EA", str_rp3_names[op & 3]);
			PRINT_DESC("%s = EA;", str_rp3_names[op & 3]);
		break;

		case 0x03:		// DCX rp
		case 0x13:
		case 0x23:
		case 0x33:
			tmp8 = (op >> 4) & 3;
			switch( tmp8 )
			{
				case 0:
					cpu.SP--;
				break;
				case 1:
					tmp16 = GET_BC - 1;
					SET_BC(tmp16);
				break;
				case 2:
					tmp16 = GET_DE - 1;
					SET_DE(tmp16);
				break;
				case 3:
					tmp16 = GET_HL - 1;
					SET_HL(tmp16);
				break;
				default:
					printf("\r\n%sUnknown DCX opcode: 0x%02X at PC=0x%04X%s\n", RED, op, cpu.PC - 1, RESET);
				break;
			}
			PRINT("DCX %s", str_rp_names[tmp8]);
			PRINT_DESC("%s--;", str_rp_names[tmp8]);
		break;

		case 0x26:		// ADINC A, byte
			condition_set = 1;
			tmp8 = second_byte();
			PRINT("ADINC A, 0x%02X", tmp8);
			PRINT_DESC("A += 0x%02X; if(!CY) {skip next}", tmp8);
			REG_A = add8(tmp8, REG_A, 0);
		break;

		case 0x30:		// DCRW wa
			condition_set = 1;
			offset = second_byte();
			addr = (REG_BANK[R_V] << 8) | offset;
			tmp8 = read_mem(addr);
			PRINT("DCRW 0x%02X", offset);
			PRINT_DESC("@0x%04X--; if( @0x%04X < 0 ) {skip next}", addr, addr);
			tmp8 = sub8(tmp8, 1, 0);
		break;

		case 0x31:		// BLOCK
			tmp16 = REG_C + 1;
			PRINT("BLOCK");
			PRINT_DESC("COPY %u bytes from (0x%04X++ <-- 0x%04X++)",  tmp16, get_rpa(RPA_DE), get_rpa(RPA_HL));
			SET_HL((GET_HL + tmp16));
			SET_DE((GET_DE + tmp16));
			REG_C = 0xFF;
		break;

		case 0x40:		// CALL
			tmp8 = second_byte();
			tmp8_1 = third_byte();
			addr = (tmp8_1 << 8) | tmp8;

			add_label(cpu.PC - 3, addr, TYPE_FUNCTION);

			tmp32 = find_label(addr);
			if(tmp32){
				PRINT("CALL %s", labels[tmp32 - 1].label);
				PRINT_DESC("%s();", labels[tmp32 - 1].label);
			} else {
				PRINT("CALL @0x%04X", cpu.PC);
				PRINT_DESC("0x%04X();", cpu.PC);
			}

			push_registers(addr);
		break;

		case 0x46:		// ADI A, byte (Add immediate with cary)
			tmp8 = second_byte();
			PRINT("ADI A, 0x%02X", tmp8);
			PRINT_DESC("A += 0x%02X;", tmp8);
		break;

		case 0x48:
			GET_CODE_FLG;
			op2 = second_byte();
			switch (op2) {
				case 0x2B:		// STC
					PRINT("STC");
					PRINT_DESC("CY = 1;");
				break;
				case 0x3A:		// NEGA
					PRINT("NEGA A");
					PRINT_DESC("A = ~A + 1;");
					REG_A = (~REG_A) + 1;
				break;
				case 0x28:		// JEA
					loc_pc = cpu.PC;
					cpu.PC = REG_EA;
					add_label(loc_pc - 2, cpu.PC, TYPE_JUMP);

					tmp32 = find_label(addr);
					if(tmp32){
						PRINT("JEA %s", labels[tmp32 - 1].label);
						PRINT_DESC("goto %s;", labels[tmp32 - 1].label);
					} else {
						PRINT("JEA @0x%04X", cpu.PC);
						PRINT_DESC("goto 0x%04X;", cpu.PC);
					}

					if(condition_set)
					{
						condition_set = 0;
						push_registers(cpu.PC);
						cpu.PC = loc_pc;
					}
				break;
				case 0x3B:		// HLT
					PRINT("HLT");
					PRINT_DESC("int halt = 1; while(halt){}");
				break;
				case 0x38:		// RLD
					tmp8 = read_mem(GET_HL);
					tmp8_1 = tmp8 >> 4;
					tmp8 = ((tmp8 << 4) & 0xF0) | (REG_A & 0x0F);
					REG_A = (REG_A & 0xF0) | (tmp8_1 & 0x0F);
					PRINT("RLD");
					PRINT_DESC("x = @0x%04X; y = (x >> 4) & 0x0F; x = (x << 4) | (A & 0x0F); A = (A & 0xF0) | y; @0x%04X = x;", GET_HL, GET_HL);
				break;
				case 0x39:		// RRD
					tmp8 = read_mem(GET_HL);
					tmp8_1 = (REG_A & 0x0F) << 4;
					REG_A = (REG_A & 0xF0 ) | (tmp8 & 0x0F);
					tmp8 = (tmp8 >> 4) | tmp8_1;
					PRINT("RRD");
					PRINT_DESC("x = @0x%04X; y = x & 0x0F; x = (x >> 4) | ((A & 0x0F) << 4); A = (A & 0xF0) | y; @0x%04X = x;", GET_HL, GET_HL);
				break;
				case 0xBB:		// STOP
					PRINT("STOP");
					PRINT_DESC("int stop = 1; while(halt){}");
				break;
				case 0x36:		// RLL r2
				case 0x35:
				case 0x37:
					tmp8_1 = CY;
					tmp8 = REG_BANK[op2 & 3];
					SET_CY((tmp8 & 0x80) ? 1 : 0);
					REG_BANK[op2 & 3] = (tmp8 << 1) | ( tmp8_1 ? 1 : 0);
					PRINT("RLL %s", str_r_names[op2 & 3]);
					PRINT_DESC("tmp = CY; CY = %s & 0x80; %s = (%s << 1) | tmp;", str_r_names[op2 & 3], str_r_names[op2 & 3], str_r_names[op2 & 3]);
				break;
				case 0x2D:		// MUL r2
				case 0x2E:
				case 0x2F:
					tmp8 = REG_BANK[op2 & 3];
					REG_EA = REG_A * tmp8;
					PRINT("MUL %s", str_r_names[op2 & 3]);
					PRINT_DESC("EA = A * %s;", str_r_names[op2 & 3]);
				break;
				case 0x92:		// STEAX rpa3
				case 0x93:
				case 0x94:
				case 0x95:
				case 0x9C:
				case 0x9D:
				case 0x9E:
					addr = get_rpa3(op2 & 0xF, 0);
					PRINT("STEAX {%s}", str_rpa3_names[op2 & 0xF]);
					PRINT_DESC("@{%s} = EAL; @{%s + 1} = EAH;", str_rpa3_names[op2 & 0xF], str_rpa3_names[op2 & 0xF]);
				break;
				case 0x9B:
				case 0x9F:
					tmp8 = third_byte();
					addr = get_rpa3(op2 & 0xF, tmp8);
					PRINT("STEAX {%s}", str_rpa3_names[op2 & 0xF]);
					PRINT_DESC("@{%s} = EAL; @{%s + 1} = EAH;", str_rpa3_names[op2 & 0xF], str_rpa3_names[op2 & 0xF]);
				break;
				case 0x82:		// LDEAX rpa3
				case 0x83:
				case 0x84:
				case 0x85:
				case 0x8C:
				case 0x8D:
				case 0x8E:
					addr = get_rpa3(op2 & 0xF, 0);
					tmp8 = read_mem(addr);
					tmp8_1 = read_mem(addr + 1);
					REG_EA = (tmp8_1 << 8) | tmp8;
					PRINT("LDEAX {%s}", str_rpa3_names[op2 & 0xF]);
					PRINT_DESC("EAL = @{%s}; EAH = @{%s + 1};", str_rpa3_names[op2 & 0xF], str_rpa3_names[op2 & 0xF]);
				break;
				case 0x8B:
				case 0x8F:
					tmp8 = third_byte();
					addr = get_rpa3(op2 & 0xF, tmp8);
					tmp8 = read_mem(addr);
					tmp8_1 = read_mem(addr + 1);
					REG_EA = (tmp8_1 << 8) | tmp8;
					PRINT("LDEAX {%s}", str_rpa3_names[op2 & 0xF]);
					PRINT_DESC("EAL = @{%s}; EAH = @{%s + 1};", str_rpa3_names[op2 & 0xF], str_rpa3_names[op2 & 0xF]);
				break;
				case 0xA8:		// TABLE
					table_instruction = 1;
					addr = cpu.PC + 1;
					REG_C = read_mem(addr);
					addr++;
					REG_B = read_mem(addr);
					PRINT("TABLE {A}");
					PRINT_DESC("Load BC with content of table below next instruction!");
				break;
				case 0x31:		// RLR r2
				case 0x32:
				case 0x33:
					tmp8 = REG_BANK[op2 & 3];
					tmp8_1 = tmp8 & 1;
					tmp8 = (tmp8 >> 1) | ( CY ? 0x80 : 0);
					SET_CY(tmp8_1);
					REG_BANK[op2 & 3] = tmp8;
					PRINT("RLR %s", str_r_names[op2 & 3]);
					PRINT_DESC("tmp = CY; CY = (%s & 1); %s = (tmp ? 0x80 : 0) | (%s >> 1);", str_r_names[op2 & 3], str_r_names[op2 & 3], str_r_names[op2 & 3]);
				break;
				case 0x05:		// SLLC r2
				case 0x06:
				case 0x07:
					condition_set = 1;
					tmp8 = REG_BANK[op2 & 3];
					SET_CY(tmp8 & 0x80);
					REG_BANK[op2 & 3] = tmp8 << 1;
					PRINT("SLLC %s", str_r_names[op2 & 3]);
					PRINT_DESC("CY = (%s & 0x80) ? 1 : 0; %s = %s << 1; if(CY) {skip next}", str_r_names[op2 & 3], str_r_names[op2 & 3], str_r_names[op2 & 3]);
				break;
				case 0x25:		// SLL r2
				case 0x26:
				case 0x27:
					tmp8 = REG_BANK[op2 & 3];
					SET_CY(tmp8 & 0x80);
					REG_BANK[op2 & 3] = tmp8 << 1;
					PRINT("SLL %s", str_r_names[op2 & 3]);
					PRINT_DESC("CY = (%s & 0x80) : 1 : 0; %s = %s << 1;", str_r_names[op2 & 3], str_r_names[op2 & 3], str_r_names[op2 & 3]);
				break;
				case 0x21:		// SLR r2
				case 0x22:
				case 0x23:
					tmp8 = REG_BANK[op2 & 3];
					SET_CY(tmp8 & 0x01);
					REG_BANK[op2 & 3] = tmp8 >> 1;
					PRINT("SLR %s", str_r_names[op2 & 3]);
					PRINT_DESC("CY = %s & 0x1; %s = %s >> 1;", str_r_names[op2 & 3], str_r_names[op2 & 3], str_r_names[op2 & 3]);
				break;
				case 0x01:		// SLRC r2
				case 0x02:
				case 0x03:
					condition_set = 1;
					tmp8 = REG_BANK[op2 & 3];
					SET_CY(tmp8 & 0x01);
					REG_BANK[op2 & 3] = tmp8 >> 1;
					PRINT("SLRC %s", str_r_names[op2 & 3]);
					PRINT_DESC("CY = %s & 1; %s = %s >> 1; if(CY) {skip next}", str_r_names[op2 & 3], str_r_names[op2 & 3], str_r_names[op2 & 3]);
				break;
				case 0x0A:		// SK f
				case 0x0B:
				case 0x0C:
					condition_set = 1;
					tmp8 = op2 & 0x7;
					switch(tmp8)
					{
						case 2:
							if(CY) SET_SK(1);
							desc = "CY";
						break;
						case 3:
							if(HC) SET_SK(1);
							desc = "HC";
						break;
						case 4:
							if(Z) SET_SK(1);
							desc = "Z";
						break;
						default:
							desc = "f flag error";
							printf("\r\n Flag error!!!\r\n");
						break;
					}
					PRINT("SK %s", desc);
					PRINT_DESC("if(%s) {skip next}", desc);
				break;
				case 0x1A:		// SKN f
				case 0x1B:
				case 0x1C:
					condition_set = 1;
					char *desc = "Flag Error";
					tmp8 = op2 & 0x7;
					SET_L0(0); SET_L1(0); SET_SK(0);
					switch(tmp8)
					{
						case 2:
							if(!CY) SET_SK(1);
							desc = "CY";
						break;
						case 3:
							if(!HC) SET_SK(1);
							desc = "HC";
						break;
						case 4:
							if(!Z) SET_SK(1);
							desc = "Z";
						break;
						default:
							printf("\r\n Flag error!!!\r\n");
						break;
					}
					PRINT("SKN %s", desc);
					PRINT_DESC("if( !%s ) {skip next}", desc);
				break;
				case 0x60:		// SKNIT irf
				case 0x61:
				case 0x62:
				case 0x63:
				case 0x64:
				case 0x65:
				case 0x66:
				case 0x67:
				case 0x68:
				case 0x69:
				case 0x6A:
				case 0x6B:
				case 0x6C:
				case 0x70:
				case 0x71:
				case 0x72:
				case 0x73:
				case 0x74:
					condition_set = 1;
					tmp8 = op2 & 0x1F;
					PRINT("SKNIT %s", str_irf_names[tmp8]);
					PRINT_DESC("if( !%s ) {skip next}", str_irf_names[tmp8]);
				break;
				case 0x40:		// SKIT irf
				case 0x41:
				case 0x42:
				case 0x43:
				case 0x44:
				case 0x45:
				case 0x46:
				case 0x47:
				case 0x48:
				case 0x49:
				case 0x4A:
				case 0x4B:
				case 0x4C:
				case 0x50:
				case 0x51:
				case 0x52:
				case 0x53:
				case 0x54:
					condition_set = 1;
					tmp8 = op2 & 0x1F;
					PRINT("SKIT %s", str_irf_names[tmp8]);
					PRINT_DESC("if(%s) {skip next}", str_irf_names[tmp8]);
				break;
				case 0xB0:		// DRLR EA
					tmp8 = REG_EA & 1;
					PRINT("DRLR EA");
					PRINT_DESC("tmp = EA & 1; EA = (EA >> 1) | CY ? 0x8000 : 0; CY = tmp; // Circular shift through Carry");
					REG_EA = (REG_EA >> 1) | (CY ? 0x8000 : 0);
				break;
				case 0xB4:		// DRLL EA
					tmp8 = CY;
					SET_CY(REG_EA & 0x8000);
					PRINT("DRLL EA");
					PRINT_DESC("tmp = CY; CY = (EA & 0x8000) ? 1 : 0; EA = (EA << 1) | tmp;  // Circular shift through Carry");
					REG_EA = (REG_EA << 1) | tmp8;
				break;
				case 0xA0:		// DSLR EA
					SET_CY(REG_EA & 1);
					PRINT("DSLR EA");
					PRINT_DESC("CY = (EA & 1); EA = EA >> 1;");
					REG_EA = REG_EA >> 1;
				break;
				case 0xA4:		// DSLL EA
					SET_CY(REG_EA & 0x8000);
					PRINT("DSLL EA");
					PRINT_DESC("CY = (EA & 0x8000) ? 1 : 0; EA = EA << 1;");
					REG_EA = REG_EA << 1;
				break;
				case 0xD2:		// DMOV sr3, EA
				case 0xD3:
					PRINT("DMOV %s, EA", (op2 & 1) ? "ETM1" : "ETM0");
					PRINT_DESC("%s = EA;", (op2 & 1) ? "ETM1" : "ETM0");
					REG_EA = (op2 & 1) ?cpu.ETM1 : cpu.ETM0;
				break;
				case 0xC0:		// DMOV EA, sr4
				case 0xC1:
					PRINT("DMOV EA, %s", (op2 & 1) ? "ECPT" : "ECNT");
					PRINT_DESC("EA = %s;", (op2 & 1) ? "ECPT" : "ECNT");
					REG_EA = (op2 & 1) ?cpu.ECPT : cpu.ECNT;
				break;
				case 0x3D:		// DIV r2
				case 0x3E:
				case 0x3F:
					tmp8 = op2 & 3;
					tmp8 = REG_BANK[op2 & 3];
					PRINT("DIV %s", str_r_names[op2 & 3]);
					PRINT_DESC("EA = EA / %s; %s = EA %% %s;", str_r_names[op2 & 3], str_r_names[op2 & 3], str_r_names[op2 & 3]);
					if( tmp8 ){
						tmp8_1 = REG_EA / tmp8;
						REG_BANK[op2 & 3] = REG_EA % tmp8;
					}
					REG_EA = tmp8_1;
				break;
				case 0x2A:	// CLC (Clear carry)
					SET_L0(0); SET_L1(0); SET_SK(0); SET_CY(0);
					PRINT("CLC (Clear Carry)");
					PRINT_DESC("CY = 0;");
				break;
				case 0x29:	// CALB
					addr = get_rpa(RPA_BC);
					add_label(cpu.PC - 2, addr, TYPE_FUNCTION);

					tmp32 = find_label(addr);
					if(tmp32){
						PRINT("CALB %s", labels[tmp32 - 1].label);
						PRINT_DESC("%s();", labels[tmp32 - 1].label);
					} else {
						PRINT("CALB @0x%04X", cpu.PC);
						PRINT_DESC("0x%04X();", cpu.PC);
					}

					push_registers(addr);
				break;
				default:
					printf("\r\nUnsupported instruction (0x%02X :: 0x%02X)\r\n", op, op2);
				break;
			};
		break;

		case 0x78:		// CALF word
		case 0x79:
		case 0x7A:
		case 0x7B:
		case 0x7C:
		case 0x7D:
		case 0x7E:
		case 0x7F:
			tmp8 = second_byte();
			addr = ((op & 0xF) << 8) | tmp8;

			add_label(cpu.PC - 2, addr, TYPE_FUNCTION);

			tmp32 = find_label(addr);
			if(tmp32){
				PRINT("CALF %s", labels[tmp32 - 1].label);
				PRINT_DESC("%s();", labels[tmp32 - 1].label);
			} else {
				PRINT("CALF @0x%04X", cpu.PC);
				PRINT_DESC("0x%04X();", cpu.PC);
			}

			push_registers(addr);
		break;

		case 0x51:		// DCR r2
		case 0x52:
		case 0x53:
			condition_set = 1;
			REG_BANK[op & 3] = sub8(REG_BANK[op & 3], 1, 0);
			PRINT("DCR %s", str_r_names[op & 3]);
			PRINT_DESC("%s--; if(%s < 0) {skip next}", str_r_names[op & 3], str_r_names[op & 3]);
		break;

		case 0x56: // ACI A, byte (Add immediate with cary)
			tmp8 = second_byte();
			PRINT("ACI A, 0x%02X", tmp8);
			PRINT_DESC("A += (0x%02X + CY);", tmp8);
			REG_A = add8(tmp8, REG_A, CY);
		break;

		case 0xA9:		// DCX EA
			REG_EA--;
			PRINT("DCX EA");
			PRINT_DESC("EA--;");
		break;

		case 0x58:		// BIT bit, wa
		case 0x59:
		case 0x5A:
		case 0x5B:
		case 0x5C:
		case 0x5D:
		case 0x5E:
		case 0x5F:
			condition_set = 1;
			offset = second_byte();
			addr = (REG_BANK[R_V] << 8) | offset;
			tmp8 = read_mem(addr);
			PRINT("BIT %u, @0x%04X", (op & 7), addr);
			PRINT_DESC("if( @0x%04X & %u ) {skip next}", addr, (op & 7));
		break;

		case 0x60: // Multiple instructions group
			GET_CODE_FLG;
			op2 = second_byte();
			switch (op2) {
				case 0x28:	// GTA r, A
				case 0x29:
				case 0x2A:
				case 0x2B:
				case 0x2C:
				case 0x2D:
				case 0x2E:
				case 0x2F:
					condition_set = 1;
					PRINT("GTA %s, A", str_r_names[op2 & 7]);
					PRINT_DESC("if(%s > A) {skip next}", str_r_names[op2 & 7]);
				break;
				case 0xA8:	// GTA A, r
				case 0xA9:
				case 0xAA:
				case 0xAB:
				case 0xAC:
				case 0xAD:
				case 0xAE:
				case 0xAF:
					condition_set = 1;
					PRINT("GTA A, %s", str_r_names[op2 & 7]);
					PRINT_DESC("if(A > %s) {skip next}", str_r_names[op2 & 7]);
				break;
				case 0xB8:	// LTA A, r
				case 0xB9:
				case 0xBA:
				case 0xBB:
				case 0xBC:
				case 0xBD:
				case 0xBE:
				case 0xBF:
					condition_set = 1;
					PRINT("LTA A, %s", str_r_names[op2 & 7]);
					PRINT_DESC("if(A < %s) {skip next}", str_r_names[op2 & 7]);
				break;
				case 0x38:	// LTA r, A
				case 0x39:
				case 0x3A:
				case 0x3B:
				case 0x3C:
				case 0x3D:
				case 0x3E:
				case 0x3F:
					condition_set = 1;
					PRINT("LTA %s, A", str_r_names[op2 & 7] );
					PRINT_DESC("if(%s < A) {skip next}", str_r_names[op2 & 7]);
				break;
				case 0x68:	// NEA r, A
				case 0x69:
				case 0x6A:
				case 0x6B:
				case 0x6C:
				case 0x6D:
				case 0x6E:
				case 0x6F:
					condition_set = 1;
					PRINT("NEA %s, A", str_r_names[op2 & 7]);
					PRINT_DESC("if(%s != A) {skip next]", str_r_names[op2 & 7]);
				break;
				case 0xC8:	// ONA A, r
				case 0xC9:
				case 0xCA:
				case 0xCB:
				case 0xCC:
				case 0xCD:
				case 0xCE:
				case 0xCF:
					condition_set = 1;
					PRINT("ONA A, %s", str_r_names[op2 & 7]);
					PRINT_DESC("if(%s & A) {skip next]", str_r_names[op2 & 7]);
				break;
				case 0xD8:	// OFFA A, r
				case 0xD9:
				case 0xDA:
				case 0xDB:
				case 0xDC:
				case 0xDD:
				case 0xDE:
				case 0xDF:
					condition_set = 1;
					PRINT("OFFA A, %s", str_r_names[op2 & 7]);
					PRINT_DESC("if(0 == (%s & A)) {skip next]", str_r_names[op2 & 7]);
				break;
				case 0xE8:	// NEA A, r
				case 0xE9:
				case 0xEA:
				case 0xEB:
				case 0xEC:
				case 0xED:
				case 0xEE:
				case 0xEF:
					condition_set = 1;
					PRINT("NEA A, %s", str_r_names[op2 & 7]);
					PRINT_DESC("if(A != %s) {skip next}", str_r_names[op2 & 7]);
				break;
				case 0x78:	// EQA r, A
				case 0x79:
				case 0x7A:
				case 0x7B:
				case 0x7C:
				case 0x7D:
				case 0x7E:
				case 0x7F:
					condition_set = 1;
					PRINT("EQA %s, A", str_r_names[op2 & 7]);
					PRINT_DESC("if(%s == A) {skip next}", str_r_names[op2 & 7]);
				break;
				case 0xF8:	// EQA A, r
				case 0xF9:
				case 0xFA:
				case 0xFB:
				case 0xFC:
				case 0xFD:
				case 0xFE:
				case 0xFF:
					condition_set = 1;
					PRINT("EQA A, %s", str_r_names[op2 & 7]);
					PRINT_DESC("if(A == %s) {skip next}", str_r_names[op2 & 7]);
				break;
				case 0x18:	// ORA r, A
				case 0x19:
				case 0x1A:
				case 0x1B:
				case 0x1C:
				case 0x1D:
				case 0x1E:
				case 0x1F:
					PRINT("ORA %s, A", str_r_names[op2 & 7]);
					PRINT_DESC("%s |= A;", str_r_names[op2 & 7]);
					REG_BANK[op2 & 7] |= REG_A;
				break;
				case 0x98:	// ORA A, r
				case 0x99:
				case 0x9A:
				case 0x9B:
				case 0x9C:
				case 0x9D:
				case 0x9E:
				case 0x9F:
					PRINT("ORA A, %s", str_r_names[op2 & 7]);
					PRINT_DESC("A |= %s;", str_r_names[op2 & 7]);
					REG_A |= REG_BANK[op2 & 7];
				break;
				case 0x08:	// ANA r, A
				case 0x09:
				case 0x0A:
				case 0x0B:
				case 0x0C:
				case 0x0D:
				case 0x0E:
				case 0x0F:
					PRINT("ANA %s, A", str_r_names[op2 & 7]);
					PRINT_DESC("%s &= A;", str_r_names[op2 & 7]);
					REG_BANK[op2 & 7] &= REG_A;
				break;
				case 0x60:	// SUB r, A
				case 0x61:
				case 0x62:
				case 0x63:
				case 0x64:
				case 0x65:
				case 0x66:
				case 0x67:
					PRINT("SUB %s, A", str_r_names[op2 & 7]);
					PRINT_DESC("%s -= A;", str_r_names[op2 & 7]);
					REG_BANK[op2 & 7] = sub8(REG_BANK[op2 & 7], REG_A, CY);
				break;
				case 0x70:	// SBB r, A
				case 0x71:
				case 0x72:
				case 0x73:
				case 0x74:
				case 0x75:
				case 0x76:
				case 0x77:
					PRINT("SBB %s, A", str_r_names[op2 & 7]);
					PRINT_DESC("%s -= (A + CY);", str_r_names[op2 & 7]);
					REG_BANK[op2 & 7] = sub8(REG_BANK[op2 & 7], REG_A, CY);
				break;
				case 0x88:	// ANA A, r
				case 0x89:
				case 0x8A:
				case 0x8B:
				case 0x8C:
				case 0x8D:
				case 0x8E:
				case 0x8F:
					PRINT("ANA A, %s", str_r_names[op2 & 7]);
					PRINT_DESC("A &= %s;", str_r_names[op2 & 7]);
					REG_A &= REG_BANK[op2 & 7];
				break;
				case 0x10:	// XRA r, A
				case 0x11:
				case 0x12:
				case 0x13:
				case 0x14:
				case 0x15:
				case 0x16:
				case 0x17:
					PRINT("XRA %s, A", str_r_names[op2 & 7]);
					PRINT_DESC("%s ^= A;", str_r_names[op2 & 7]);
					REG_BANK[op2 & 7] ^= REG_A;
				break;
				case 0x90:	// XRA A, r
				case 0x91:
				case 0x92:
				case 0x93:
				case 0x94:
				case 0x95:
				case 0x96:
				case 0x97:
					PRINT("XRA A, %s", str_r_names[op2 & 7]);
					PRINT_DESC("A ^= %s;", str_r_names[op2 & 7]);
					REG_A ^= REG_BANK[op2 & 7];
				break;
				case 0xB0:	// SUBNB A, r
				case 0xB1:
				case 0xB2:
				case 0xB3:
				case 0xB4:
				case 0xB5:
				case 0xB6:
				case 0xB7:
					condition_set = 1;
					PRINT("SUBNB A, %s", str_r_names[op2 & 7]);
					PRINT_DESC("A -= %s; if(A >= 0) {skip next}", str_r_names[op2 & 7]);
					REG_A = sub8(REG_A, REG_BANK[op2 & 7], 0);
				break;
				case 0x30:	// SUBNB r, A
				case 0x31:
				case 0x32:
				case 0x33:
				case 0x34:
				case 0x35:
				case 0x36:
				case 0x37:
					condition_set = 1;
					PRINT("SUBNB %s, A", str_r_names[op2 & 7]);
					PRINT_DESC("%s -= A; if(A >= 0) {skip next}", str_r_names[op2 & 7]);
					REG_BANK[op2 & 7] = sub8(REG_BANK[op2 & 7], REG_A, 0);
				break;
				case 0xE0:	// SUB A, r
				case 0xE1:
				case 0xE2:
				case 0xE3:
				case 0xE4:
				case 0xE5:
				case 0xE6:
				case 0xE7:
					PRINT("SUB A, %s", str_r_names[op2 & 7]);
					PRINT_DESC("A -= %s;", str_r_names[op2 & 7]);
					REG_A = sub8(REG_A, REG_BANK[op2 & 7], 0);
				break;
				case 0xF0:	// SBB A, r
				case 0xF1:
				case 0xF2:
				case 0xF3:
				case 0xF4:
				case 0xF5:
				case 0xF6:
				case 0xF7:
					PRINT("SBB A, %s", str_r_names[op2 & 7]);
					PRINT_DESC("A -= (%s + CY);", str_r_names[op2 & 7]);
					REG_A = sub8(REG_A, REG_BANK[op2 & 7], CY);
				break;
				case 0x40:	// ADD r, A
				case 0x41:
				case 0x42:
				case 0x43:
				case 0x44:
				case 0x45:
				case 0x46:
				case 0x47:
					PRINT("ADD %s, A", str_r_names[op2 & 7]);
					PRINT_DESC("%s += A;", str_r_names[op2 & 7]);
					REG_BANK[op2 & 7] = add8(REG_BANK[op2 & 7], REG_A, 0);
				break;
				case 0xC0:	// ADD A, r
				case 0xC1:
				case 0xC2:
				case 0xC3:
				case 0xC4:
				case 0xC5:
				case 0xC6:
				case 0xC7:
					PRINT("ADD A, %s", str_r_names[op2 & 7]);
					PRINT_DESC("A += %s;", str_r_names[op2 & 7]);
					REG_A = add8(REG_BANK[op2 & 7], REG_A, 0);
				break;
				case 0xD0:	// ADC A, r
				case 0xD1:
				case 0xD2:
				case 0xD3:
				case 0xD4:
				case 0xD5:
				case 0xD6:
				case 0xD7:
					PRINT("ADC A, %s", str_r_names[op2 & 7]);
					PRINT_DESC("A = A + %s + CY", str_r_names[op2 & 7]);
					REG_A = add8(REG_BANK[op2 & 7], REG_A, CY);
				break;
				case 0x50:	// ADC r, A
				case 0x51:
				case 0x52:
				case 0x53:
				case 0x54:
				case 0x55:
				case 0x56:
				case 0x57:
					PRINT("ADC %s, A", str_r_names[op2 & 7]);
					PRINT_DESC(" %s += (A + CY)", str_r_names[op2 & 7]);
					REG_BANK[op2 & 7] = add8(REG_BANK[op2 & 7], REG_A, CY);
				break;
				case 0xA0:	// ADDNC A, r
				case 0xA1:
				case 0xA2:
				case 0xA3:
				case 0xA4:
				case 0xA5:
				case 0xA6:
				case 0xA7:
					condition_set = 1;
					PRINT("ADDNC A, %s", str_r_names[op2 & 7]);
					PRINT_DESC("A += %s; if(A > 0xFF) {skip next}", str_r_names[op2 & 7]);
					REG_A = add8(REG_BANK[op2 & 7], REG_A, 0);
				break;
				case 0x20:	// ADDNC r, A
				case 0x21:
				case 0x22:
				case 0x23:
				case 0x24:
				case 0x25:
				case 0x26:
				case 0x27:
					condition_set = 1;
					PRINT("ADDNC %s, A", str_r_names[op2 & 7]);
					PRINT_DESC("%s += A; if(%s > 0xFF) {skip next}", str_r_names[op2 & 7], str_r_names[op2 & 7]);
					REG_BANK[op2 & 7] = add8(REG_BANK[op2 & 7], REG_A, 0);
				break;
				default:
					printf("\r\nUnsupported instruction (0x%02X :: 0x%02X)\r\n", op, op2);
				break;
			};
		break;

		case 0x61:		// DAA (decimal adjustment)
			tmp8 = (REG_A & 0xF);
			tmp8_1 = ((REG_A >> 4) & 0xF);
			src = REG_A;
			if( (tmp8 < 10) && (0 == HC) )
			{
				if( (tmp8_1 > 9) || (1 == CY) )
				{
					tmp16 = REG_A + 0x60;
					SET_HC((tmp16 ^ REG_A ^ 0x60) & 0x10);
					SET_CY(tmp16 & 0xFF00);
					SET_Z(tmp16 & 0xFF);
				}
			}
			else if( (tmp8 > 9) && (0 == HC) )
			{
				if( (tmp8_1 > 8) || (1 == CY) )
				{
					tmp16 = REG_A + 0x66;
					SET_HC((tmp16 ^ REG_A ^ 0x66) & 0x10);
					SET_CY(tmp16 & 0xFF00);
					SET_Z(tmp16 & 0xFF);
				}
				else if( (tmp8_1 < 9) && (0 == CY) )
				{
					tmp16 = REG_A + 0x06;
					SET_HC((tmp16 ^ REG_A ^ 0x06) & 0x10);
					SET_CY(tmp16 & 0xFF00);
					SET_Z(tmp16 & 0xFF);
				}
			}
			else if( (tmp8 < 3) && (1 == HC) )
			{
				if( (tmp8_1 > 9) || (1 == CY) )
				{
					tmp16 = REG_A + 0x66;
					SET_HC((tmp16 ^ REG_A ^ 0x66) & 0x10);
					SET_CY(tmp16 & 0xFF00);
					SET_Z(tmp16 & 0xFF);
				}
				else if( (tmp8_1 < 10) && (0 == CY) )
				{
					tmp16 = REG_A + 0x06;
					SET_HC((tmp16 ^ REG_A ^ 0x06) & 0x10);
					SET_CY(tmp16 & 0xFF00);
					SET_Z(tmp16 & 0xFF);
				}
			}
			PRINT("DAA");
			PRINT_DESC("// Convert A to decimal nibbles: 0x%02X --> 0x%02X", src, REG_A);
		break;

		case 0x64: // Multiple instructions group
			GET_CODE_FLG;
			op2 = second_byte();
			switch( op2 )
			{
				case 0x30:		// SUINB sr2, byte
				case 0x31:
				case 0x32:
				case 0x33:
				case 0x35:
				case 0x36:
				case 0x37:
				case 0xB0:
				case 0xB1:
				case 0xB3:
				case 0xB5:
					condition_set = 1;
					op2 = ((op2 &0x80) >> 4) | (op2 & 0x7);
					tmp8 = third_byte();
					set_sr(op2, tmp8);
					PRINT("SUINB %s, 0x%02X", str_sr_names[op2], tmp8);
					PRINT_DESC("%s -= 0x%02X; if(%s >= 0) {skip next}", str_sr_names[op2], tmp8, str_sr_names[op2]);
				break;
				case 0x60:		// SUI sr2, byte
				case 0x61:
				case 0x62:
				case 0x63:
				case 0x65:
				case 0x66:
				case 0x67:
				case 0xE0:
				case 0xE1:
				case 0xE3:
				case 0xE5:
					op2 = ((op2 &0x80) >> 4) | (op2 & 0x7);
					tmp8 = third_byte();
					set_sr(op2, tmp8);
					PRINT("SUI %s, 0x%02X", str_sr_names[op2], tmp8);
					PRINT_DESC("%s -= 0x%02X;", str_sr_names[op2], tmp8);
				break;
				case 0x70:		// SBI sr2, byte
				case 0x71:
				case 0x72:
				case 0x73:
				case 0x75:
				case 0x76:
				case 0x77:
				case 0xF0:
				case 0xF1:
				case 0xF3:
				case 0xF5:
					op2 = ((op2 &0x80) >> 4) | (op2 & 0x7);
					tmp8 = third_byte();
					set_sr(op2, tmp8);
					PRINT("SBI %s, 0x%02X", str_sr_names[op2], tmp8);
					PRINT_DESC("%s -= (0x%02X + CY);", str_sr_names[op2], tmp8);
				break;
				case 0x38:		// LTI sr2, byte
				case 0x39:
				case 0x3A:
				case 0x3B:
				case 0x3D:
				case 0x3E:
				case 0x3F:
				case 0xB8:
				case 0xB9:
				case 0xBB:
				case 0xBD:
					condition_set = 1;
					op2 = ((op2 &0x80) >> 4) | (op2 & 0x7);
					tmp8 = third_byte();
					PRINT("LTI %s, 0x%02X", str_sr_names[op2], tmp8);
					PRINT_DESC("if( %s < 0x%02X) {skip next}", str_sr_names[op2], tmp8);
				break;
				case 0x28:		// GTI sr2, byte
				case 0x29:
				case 0x2A:
				case 0x2B:
				case 0x2D:
				case 0x2E:
				case 0x2F:
				case 0xA8:
				case 0xA9:
				case 0xAB:
				case 0xAD:
					condition_set = 1;
					op2 = ((op2 &0x80) >> 4) | (op2 & 0x7);
					tmp8 = third_byte();
					PRINT("GTI %s, 0x%02X", str_sr_names[op2], tmp8);
					PRINT_DESC("if(%s > 0x%02X) {skip next}", str_sr_names[op2], tmp8);
				break;
				case 0x18:		// ORI sr2, byte
				case 0x19:
				case 0x1A:
				case 0x1B:
				case 0x1D:
				case 0x1E:
				case 0x1F:
				case 0x98:
				case 0x99:
				case 0x9B:
				case 0x9D:
					op2 = ((op2 &0x80) >> 4) | (op2 & 0x7);
					tmp8 = third_byte();
					set_sr(op2, get_sr(op2) | tmp8);
					PRINT("ORI %s, 0x%02X", str_sr_names[op2], tmp8);
					PRINT_DESC("%s |= 0x%02X;", str_sr_names[op2], tmp8);
				break;
				case 0x58:		// OFFI sr2, byte
				case 0x59:
				case 0x5A:
				case 0x5B:
				case 0x5D:
				case 0x5E:
				case 0x5F:
				case 0xD8:
				case 0xD9:
				case 0xDB:
				case 0xDD:
					condition_set = 1;
					op2 = ((op2 &0x80) >> 4) | (op2 & 0x7);
					tmp8 = third_byte();
					PRINT("OFFI %s, 0x%02X", str_sr_names[op2], tmp8);
					PRINT_DESC("if(0 ==(%s & 0x%02X)) {skip next}", str_sr_names[op2], tmp8);
				break;
				case 0x48:		// ONI sr2, byte
				case 0x49:
				case 0x4A:
				case 0x4B:
				case 0x4D:
				case 0x4E:
				case 0x4F:
				case 0xC8:
				case 0xC9:
				case 0xCB:
				case 0xCD:
					condition_set = 1;
					op2 = ((op2 &0x80) >> 4) | (op2 & 0x7);
					tmp8 = third_byte();
					tmp8_1 = get_sr(op2);
					PRINT("ONI %s, 0x%02X", str_sr_names[op2], tmp8);
					PRINT_DESC("if(%s & 0x%02X) {skip next}", str_sr_names[op2], tmp8);
				break;
				case 0x00:		// MVI sr2, byte
				case 0x01:
				case 0x02:
				case 0x03:
				case 0x05:
				case 0x06:
				case 0x07:
				case 0x80:
				case 0x81:
				case 0x83:
				case 0x85:
					op2 = ((op2 &0x80) >> 4) | (op2 & 0x7);
					tmp8 = third_byte();
					set_sr(op2, tmp8);
					PRINT("MVI %s, 0x%02X", str_sr_names[op2], tmp8);
					PRINT_DESC("%s = 0x%02X;", str_sr_names[op2], tmp8);
				break;
				case 0x68:		// NEI sr2, byte
				case 0x69:
				case 0x6A:
				case 0x6B:
				case 0x6D:
				case 0x6E:
				case 0x6F:
				case 0xE8:
				case 0xE9:
				case 0xEB:
				case 0xED:
					condition_set = 1;
					op2 = ((op2 &0x80) >> 4) | (op2 & 0x7);
					tmp8 = third_byte();
					PRINT("NEI %s, 0x%02X", str_sr_names[op2], tmp8);
					PRINT_DESC("if(%s != 0x%02X) {skip next}", str_sr_names[op2], tmp8);
				break;
				case 0x78:		// EQI sr2, byte
				case 0x79:
				case 0x7A:
				case 0x7B:
				case 0x7D:
				case 0x7E:
				case 0x7F:
				case 0xF8:
				case 0xF9:
				case 0xFB:
				case 0xFD:
					condition_set = 1;
					op2 = ((op2 &0x80) >> 4) | (op2 & 0x7);
					tmp8 = third_byte();
					PRINT("EQI %s, 0x%02X", str_sr_names[op2], tmp8);
					PRINT_DESC("if(%s == 0x%02X) {skip next}", str_sr_names[op2], tmp8);
				break;
				case 0x10:		// XRI sr2, byte
				case 0x11:
				case 0x12:
				case 0x13:
				case 0x15:
				case 0x16:
				case 0x17:
				case 0x90:
				case 0x91:
				case 0x93:
				case 0x95:
					op2 = ((op2 &0x80) >> 4) | (op2 & 0x7);
					tmp8 = third_byte();
					PRINT("XRI %s, 0x%02X", str_sr_names[op2], op2);
					PRINT_DESC("%s ^= 0x%02X;", str_sr_names[op2], op2);
					set_sr(op2, get_sr(op2) ^ tmp8);
				break;
				case 0x50:		// ACI sr2, byte (Add immediate with cary)
				case 0x51:
				case 0x52:
				case 0x53:
				case 0x55:
				case 0x56:
				case 0x57:
				case 0xD0:
				case 0xD1:
				case 0xD3:
				case 0xD5:
					op2 = ((op2 &0x80) >> 4) | (op2 & 0x7);
					tmp8 = third_byte();
					PRINT("ACI %s, 0x%02X", str_sr_names[op2], op2);
					PRINT_DESC("%s += (0x%02X + CY);", str_sr_names[op2], op2);
					set_sr(op2, add8(get_sr(op2), tmp8, CY));
				break;
				case 0x40:		// ADI sr2, byte
				case 0x41:
				case 0x42:
				case 0x43:
				case 0x45:
				case 0x46:
				case 0x47:
				case 0xC0:
				case 0xC1:
				case 0xC3:
				case 0xC5:
					op2 = ((op2 &0x80) >> 4) | (op2 & 0x7);
					tmp8 = third_byte();
					PRINT("ADI %s, 0x%02X", str_sr_names[op2], op2);
					PRINT_DESC("%s += 0x%02X;", str_sr_names[op2], op2);
					set_sr(op2, add8(get_sr(op2), tmp8, 0));
				break;
				case 0x20:		// ADINC sr2, byte
				case 0x21:
				case 0x22:
				case 0x23:
				case 0x25:
				case 0x26:
				case 0x27:
				case 0xA0:
				case 0xA1:
				case 0xA3:
				case 0xA5:
					condition_set = 1;
					op2 = ((op2 &0x80) >> 4) | (op2 & 0x7);
					tmp8 = third_byte();
					PRINT("ADINC %s, 0x%02X", str_sr_names[op2], op2);
					PRINT_DESC("%s += 0x%02X; if( !CY ) {skip next}", str_sr_names[op2], op2);
					set_sr(op2, add8(get_sr(op2), tmp8, 0));
				break;
				case 0x08:		// ANI sr2, byte
				case 0x09:
				case 0x0A:
				case 0x0B:
				case 0x0D:
				case 0x0E:
				case 0x0F:
				case 0x88:
				case 0x89:
				case 0x8B:
				case 0x8D:
					op2 = ((op2 &0x80) >> 4) | (op2 & 0x7);
					tmp8 = third_byte();
					PRINT("ANI %s, 0x%02X", str_sr_names[op2], op2);
					PRINT_DESC("%s &= 0x%02X;", str_sr_names[op2], op2);
				break;
				default:
					printf("\r\nUnsupported instruction (0x%02X :: 0x%02X)\r\n", op, op2);
				break;
			}
		break;

		case 0x25:	// GTIW wa, byte
			condition_set = 1;
			offset = second_byte();
			addr = (REG_BANK[R_V] << 8) | offset;
			tmp8 = read_mem(addr);
			tmp8_1 = third_byte();
			PRINT("GTIW 0x%02X, 0x%02X", offset, tmp8_1);
			PRINT_DESC("if(@0x%04X > 0x%02X) {skip next}", addr, tmp8_1);
		break;

		case 0x35:	// LTIW wa, byte
			condition_set = 1;
			offset = second_byte();
			addr = (REG_BANK[R_V] << 8) | offset;
			tmp8 = read_mem(addr);
			tmp8_1 = third_byte();
			PRINT("LTIW 0x%02X", offset);
			PRINT_DESC("if(@0x%04X < 0x%02X) {skip next}", addr, tmp8_1);
		break;

		case 0x75:	// EQIW wa, byte
			condition_set = 1;
			offset = second_byte();
			addr = (REG_BANK[R_V] << 8) | offset;
			tmp8_1 = third_byte();
			PRINT("EQIW 0x%02X, 0x%02X", offset, tmp8_1);
			PRINT_DESC("if( @0x%04X == 0x%02X ) {skip next}", addr, tmp8_1);
		break;

		case 0x74:
			GET_CODE_FLG;
			op2 = second_byte();
			switch (op2) {
				case 0xe8:		// NEAW wa
					condition_set = 1;
					offset = third_byte();
					addr = (REG_BANK[R_V] << 8) | offset;
					PRINT("NEAW 0x%02X", offset);
					PRINT_DESC("if(@0x%04X != A) {skip next}", addr);
				break;
				case 0xB8:		// LTAW wa
					condition_set = 1;
					offset = third_byte();
					addr = (REG_BANK[R_V] << 8) | offset;
					PRINT("LTAW 0x%02X", offset);
					PRINT_DESC("if( A < @0x%04X) {skip next}", addr);
				break;
				case 0xA8:		// GTAW wa
					condition_set = 1;
					offset = third_byte();
					addr = (REG_BANK[R_V] << 8) | offset;
					tmp8 = read_mem(addr);
					PRINT("GTAW 0x%02X", offset);
					PRINT_DESC("if(A > @0x%04X) {skip next}", addr);
				break;
				case 0xC8:		// ONAW wa
					condition_set = 1;
					offset = third_byte();
					addr = (REG_BANK[R_V] << 8) | offset;
					tmp8 = read_mem(addr);
					PRINT("ONAW 0x%02X", offset);
					PRINT_DESC("if(A & @0x%04X) {skip next}", addr);
				break;
				case 0xD8:		// OFFAW wa
					condition_set = 1;
					offset = third_byte();
					addr = (REG_BANK[R_V] << 8) | offset;
					tmp8 = read_mem(addr);
					PRINT("OFFAW 0x%02X", offset);
					PRINT_DESC("if(0 == (A & @0x%04X)) {skip next}", addr);
				break;
				case 0x18:		// ORI r, byte
				case 0x19:
				case 0x1A:
				case 0x1B:
				case 0x1C:
				case 0x1D:
				case 0x1E:
				case 0x1F:
					tmp8 = third_byte();
					PRINT("ORI %s, 0x%02X", str_r_names[op2 & 7], tmp8);
					PRINT_DESC("%s |= 0x%02X;", str_r_names[op2 & 7], tmp8);
					REG_BANK[op2 & 7] |= tmp8;
				break;
				case 0x38:		// LTI r, byte
				case 0x39:
				case 0x3A:
				case 0x3B:
				case 0x3C:
				case 0x3D:
				case 0x3E:
				case 0x3F:
					condition_set = 1;
					tmp8 = third_byte();
					PRINT("LTI %s, 0x%02X", str_r_names[op2 & 7], tmp8);
					PRINT_DESC("if( %s < 0x%02X) {skip next}", str_r_names[op2 & 7], tmp8);
				break;
				case 0x28:		// GTI r, byte
				case 0x29:
				case 0x2A:
				case 0x2B:
				case 0x2C:
				case 0x2D:
				case 0x2E:
				case 0x2F:
					condition_set = 1;
					tmp8 = third_byte();
					tmp8_1 = (REG_BANK[op2 & 7] > tmp8) ? 1 : 0;
					PRINT("GTI %s, 0x%02X", str_r_names[op2 & 7], tmp8);
					PRINT_DESC("if(%s > 0x%02X) {skip next}", str_r_names[op2 & 7], tmp8);
				break;
				case 0x58:		// OFFI r, byte
				case 0x59:
				case 0x5A:
				case 0x5B:
				case 0x5C:
				case 0x5D:
				case 0x5E:
				case 0x5F:
					condition_set = 1;
					tmp8 = third_byte();
					PRINT("OFFI %s, 0x%02X", str_r_names[op2 & 7], tmp8);
					PRINT_DESC("if(0 == (%s & 0x%02X)) {skip next}", str_r_names[op2 & 7], tmp8);
				break;
				case 0x68:		// NEI r, byte
				case 0x69:
				case 0x6A:
				case 0x6B:
				case 0x6C:
				case 0x6D:
				case 0x6E:
				case 0x6F:
					condition_set = 1;
					tmp8 = third_byte();
					PRINT("NEI %s, 0x%02X", str_r_names[op2 & 7], tmp8);
					PRINT_DESC("if(%s != 0x%02X) {skip next}", str_r_names[op2 & 7], tmp8);
				break;
				case 0x78:		// EQI r, byte
				case 0x79:
				case 0x7A:
				case 0x7B:
				case 0x7C:
				case 0x7D:
				case 0x7E:
				case 0x7F:
					condition_set = 1;
					tmp8 = third_byte();
					PRINT("EQI %s, 0x%02X", str_r_names[op2 & 7], tmp8);
					PRINT_DESC("if(%s == 0x%02X) {skip next}", str_r_names[op2 & 7], tmp8);
				break;
				case 0x95:		// DXR EA, rp3
				case 0x96:
				case 0x97:
					tmp16 = get_rp3((op2 & 3));
					PRINT("DXR EA, %s", str_rp3_names[op2 & 3]);
					PRINT_DESC("EA ^= %s;", str_rp3_names[op2 & 3]);
					REG_EA = REG_EA ^ tmp16;
				break;
				case 0x9D:		// DOR EA, rp3
				case 0x9E:
				case 0x9F:
					tmp16 = get_rp3((op2 & 3));
					PRINT("DOR EA, %s", str_rp3_names[op2 & 3]);
					PRINT_DESC("EA |= %s;", str_rp3_names[op2 & 3]);
					tmp16 = REG_EA | tmp16;
				break;
				case 0xCD:		// DON EA, rp3
				case 0xCE:
				case 0xCF:
					condition_set = 1;
					tmp16 = get_rp3((op2 & 3));
					PRINT("DON EA, %s", str_rp3_names[op2 & 3]);
					PRINT_DESC("if(EA & %s) {skip next}", str_rp3_names[op2 & 3]);
				break;
				case 0xDD:		// DOFF EA, rp3
				case 0xDE:
				case 0xDF:
					condition_set = 1;
					tmp16 = get_rp3((op2 & 3));
					PRINT("DOFF EA, %s", str_rp3_names[op2 & 3]);
					PRINT_DESC("if(0 == (EA & %s)) {skip next}", str_rp3_names[op2 & 3]);
				break;
				case 0xED:		// DNE EA, rp3
				case 0xEE:
				case 0xEF:
					condition_set = 1;
					tmp16 = get_rp3((op2 & 3));
					PRINT("DNE EA, %s", str_rp3_names[op2 & 3]);
					PRINT_DESC("if( EA != %s ) {skip next}", str_rp3_names[op2 & 3]);
				break;
				case 0xBD:		// DLT EA, rp3
				case 0xBE:
				case 0xBF:
					condition_set = 1;
					tmp16 = get_rp3((op2 & 3));
					PRINT("DGT EA, %s", str_rp3_names[op2 & 3]);
					PRINT_DESC("if(EA > %s) {skip next}", str_rp3_names[op2 & 3]);
				break;
				case 0xAD:		// DGT EA, rp3
				case 0xAE:
				case 0xAF:
					condition_set = 1;
					tmp16 = get_rp3((op2 & 3)) + 1;
					PRINT("DGT EA, %s", str_rp3_names[op2 & 3]);
					PRINT_DESC("if(EA > %s) {skip next}", str_rp3_names[op2 & 3]);
				break;
				case 0xFD:		// DEQ EA, rp3
				case 0xFE:
				case 0xFF:
					condition_set = 1;
					tmp16 = get_rp3((op2 & 3));
					PRINT("DEQ EA, %s", str_rp3_names[op2 & 3]);
					PRINT_DESC("if(EA == %s) {skip next}", str_rp3_names[op2 & 3]);
				break;
				case 0x8D:		// DAN EA, rp3
				case 0x8E:
				case 0x8F:
					condition_set = 1;
					tmp16 = get_rp3((op2 & 3));
					PRINT("DAN EA, %s", str_rp3_names[op2 & 3]);
					PRINT_DESC("EA &= %s;", str_rp3_names[op2 & 3]);
					REG_EA &= tmp16;
				break;
				case 0xF8:		// EQAW wa
					condition_set = 1;
					offset = third_byte();
					addr = (REG_BANK[R_V] << 8) | offset;
					tmp8 = read_mem(addr);
					PRINT("EQAW 0x%02X", offset);
					PRINT_DESC("if(A == @0x%04X) {skip next}", addr);
				break;
				case 0x88:		// ANAW wa
					offset = third_byte();
					addr = (REG_BANK[R_V] << 8) | offset;
					tmp8 = read_mem(addr);
					PRINT("ANAW 0x%02X", offset);
					PRINT_DESC("A &= @0x%04X;", addr);
					REG_A = REG_A & tmp8;
				break;
				case 0x98:		// ORAW wa
					offset = third_byte();
					addr = (REG_BANK[R_V] << 8) | offset;
					tmp8 = read_mem(addr);
					PRINT("ORAW 0x%02X", offset);
					PRINT_DESC("A |= @0x%04X;", addr);
					REG_A = REG_A | tmp8;
				break;
				case 0x90:		// XRAW wa
					offset = third_byte();
					addr = (REG_BANK[R_V] << 8) | offset;
					tmp8 = read_mem(addr);
					PRINT("XRAW 0x%02X", offset);
					PRINT_DESC("A ^= @0x%04X;", addr);
					REG_A = REG_A ^ tmp8;
				break;
				case 0xA0:		// ADDNCW wa
					condition_set = 1;
					offset = third_byte();
					addr = (REG_BANK[R_V] << 8) | offset;
					tmp8 = read_mem(addr);
					PRINT("ADDNCW 0x%02X", offset);
					PRINT_DESC("A += @0x%04X; if(A > 0xFF) {skip next}", addr);
					REG_A = add8(REG_A, tmp8, CY);
				break;
				case 0xB0:		// SUBNBW wa
					condition_set = 1;
					offset = third_byte();
					addr = (REG_BANK[R_V] << 8) | offset;
					tmp8 = read_mem(addr);
					PRINT("SUBNBW 0x%02X", offset);
					PRINT_DESC("A -= @0x%04X; if(A >= 0) {skip next}", addr);
					REG_A = sub8(REG_A, tmp8, 0);
				break;
				case 0xC0:		// ADDW wa
					offset = third_byte();
					addr = (REG_BANK[R_V] << 8) | offset;
					tmp8 = read_mem(addr);
					PRINT("ADDW 0x%02X", offset);
					PRINT_DESC("A += @0x%04X;", addr);
					REG_A = add8(REG_A, tmp8, 0);
				break;
				case 0xD0:		// ADCW wa
					condition_set = 1;
					offset = third_byte();
					addr = (REG_BANK[R_V] << 8) | offset;
					tmp8 = read_mem(addr);
					PRINT("ADCW 0x%02X", offset);
					PRINT_DESC("A += (@0x%04X + CY); if(A > 0xFF) {skip next}", addr);
					REG_A = add8(REG_A, tmp8, CY);
				break;
				case 0xE0:		// SUBW wa
					offset = third_byte();
					addr = (REG_BANK[R_V] << 8) | offset;
					tmp8 = read_mem(addr);
					PRINT("SUBW A, 0x%02X", tmp8);
					PRINT_DESC("A -= @0x%04X;", addr);
					REG_A = sub8(REG_A, tmp8, 0);
				break;
				case 0xF0:		// SBBW wa
					offset = third_byte();
					addr = (REG_BANK[R_V] << 8) | offset;
					tmp8 = read_mem(addr);
					PRINT("SBBW A, 0x%02X", tmp8);
					PRINT_DESC("A -= (@0x%04X + CY);", addr);
					REG_A = sub8(REG_A, tmp8, CY);
				break;
				case 0xB5:		// DSUBNB EA, rp3
				case 0xB6:
				case 0xB7:
					condition_set = 1;
					tmp16 = get_rp3((op2 & 3));
					PRINT("DSUBNB EA, %s", str_rp3_names[op2 & 3]);
					PRINT_DESC("EA -= %s; if(EA >= 0) {skip next}", str_rp3_names[op2 & 3]);
					uint16_t tmp_res = sub16(REG_EA, tmp16, 0);
					REG_EA = tmp_res;
				break;	
				case 0xE5:		// DSUB EA, rp3
				case 0xE6:
				case 0xE7:
					tmp16 = get_rp3((op2 & 3));
					PRINT("DSUB EA, %s", str_rp3_names[op2 & 3]);
					PRINT_DESC("EA -= %s;", str_rp3_names[op2 & 3]);
					REG_EA = sub16(REG_EA, tmp16, 0);
				break;	
				case 0xF5:		// DSBB EA, rp3
				case 0xF6:
				case 0xF7:
					tmp16 = get_rp3((op2 & 3));
					PRINT("DSBB EA, %s", str_rp3_names[op2 & 3]);
					PRINT_DESC("EA -= (%s + CY);", str_rp3_names[op2 & 3]);
					REG_EA = sub16(REG_EA, tmp16, CY);
				break;				
				case 0xD5:		// DADC EA, rp3
				case 0xD6:
				case 0xD7:
					tmp16 = get_rp3((op2 & 3));
					PRINT("DADC EA, %s", str_rp3_names[op2 & 3]);
					PRINT_DESC("EA += (%s + CY);", str_rp3_names[op2 & 3]);
					REG_EA = add16(REG_EA, tmp16, CY);
				break;
				case 0xC5:		// DADD EA, rp3
				case 0xC6:
				case 0xC7:
					tmp16 = get_rp3((op2 & 3));
					PRINT("DADD EA, %s", str_rp3_names[op2 & 3]);
					PRINT_DESC("EA += %s;", str_rp3_names[op2 & 3]);
					REG_EA = add16(REG_EA, tmp16, 0);
				break;
				case 0xA5:		// DADDNC EA, rp3
				case 0xA6:
				case 0xA7:
					condition_set = 1;
					tmp16 = get_rp3((op2 & 3));
					PRINT("DADDNC EA, %s", str_rp3_names[op2 & 3]);
					PRINT_DESC("EA += %s; if(EA > 0xFFFF) {skip next}", str_rp3_names[op2 & 3]);
					REG_EA = add16(REG_EA, tmp16, 0);
				break;
				case 0x08:		// ANI r, byte
				case 0x09:
				case 0x0A:
				case 0x0B:
				case 0x0C:
				case 0x0D:
				case 0x0E:
				case 0x0F:
					tmp8 = third_byte();
					PRINT("ANI %s, 0x%02X", str_r_names[op2 & 7], tmp8);
					PRINT_DESC("%s &= 0x%02X;", str_r_names[op2 & 7], tmp8);
					REG_BANK[op2 & 7] = REG_BANK[op2 & 7] & tmp8;
				break;
				case 0x10:		// XRI r, byte
				case 0x11:
				case 0x12:
				case 0x13:
				case 0x14:
				case 0x15:
				case 0x16:
				case 0x17:
					tmp8 = third_byte();
					PRINT("XRI %s, 0x%02X", str_r_names[op2 & 7], tmp8);
					PRINT_DESC("%s ^= 0x%02X;", str_r_names[op2 & 7], tmp8, str_r_names[op2 & 7]);
					REG_BANK[op2 & 7] = REG_BANK[op2 & 7] ^ tmp8;
				break;
				case 0x20:		// ADINC r, byte (Add immediate with cary)
				case 0x21:
				case 0x22:
				case 0x23:
				case 0x24:
				case 0x25:
				case 0x26:
				case 0x27:
					condition_set = 1;
					tmp8 = third_byte();
					PRINT("ADINC %s, 0x%02X", str_r_names[op2 & 7], tmp8);
					PRINT_DESC("%s += 0x%02X; if(%s < 256) {skip next}", str_r_names[op2 & 7], tmp8, str_r_names[op2 & 7]);
					REG_BANK[op2 & 7] = add8(REG_BANK[op2 & 7], tmp8, 0);
				break;
				case 0x30:		// SUINB r, byte
				case 0x31:
				case 0x32:
				case 0x33:
				case 0x34:
				case 0x35:
				case 0x36:
				case 0x37:
					condition_set = 1;
					tmp8 = third_byte();
					PRINT("SUINB %s, 0x%02X", str_r_names[op2 & 7], tmp8);
					PRINT_DESC("%s -= 0x%02X; if(%s > 0) {skip next}", str_r_names[op2 & 7], tmp8, str_r_names[op2 & 7]);
					REG_BANK[op2 & 7] = sub8(REG_BANK[op2 & 7], tmp8, 0);
				break;
				case 0x40:		// ADI r, byte (Add immediate with cary)
				case 0x41:
				case 0x42:
				case 0x43:
				case 0x44:
				case 0x45:
				case 0x46:
				case 0x47:
					tmp8 = third_byte();
					PRINT("ADI %s, 0x%02X", str_r_names[op2 & 7], tmp8);
					PRINT_DESC("%s += 0x%02X;", str_r_names[op2 & 7], tmp8);
					REG_BANK[op2 & 7] = add8(REG_BANK[op2 & 7], tmp8, 0);
				break;
				case 0x48:		// ONI r, byte (Add immediate with cary)
				case 0x49:
				case 0x4A:
				case 0x4B:
				case 0x4C:
				case 0x4D:
				case 0x4E:
				case 0x4F:
					condition_set = 1;
					tmp8 = third_byte();
					tmp8_1 = REG_BANK[op2 & 7] & tmp8;
					PRINT("ONI %s, 0x%02X", str_r_names[op2 & 7], tmp8);
					PRINT_DESC("if(%s & 0x%02X) {skip next}", str_r_names[op2 & 7], tmp8);
				break;
				case 0x50:		// ACI r, byte (Add immediate with cary)
				case 0x51:
				case 0x52:
				case 0x53:
				case 0x54:
				case 0x55:
				case 0x56:
				case 0x57:
					tmp8 = third_byte();
					PRINT("ACI %s, 0x%02X", str_r_names[op2 & 7], tmp8);
					PRINT_DESC("%s += (0x%02X + CY);", str_r_names[op2 & 7], tmp8);
					REG_BANK[op2 & 7] = add8(REG_BANK[op2 & 7], tmp8, CY);
				break;
				case 0x60:		// SUI r, byte
				case 0x61:
				case 0x62:
				case 0x63:
				case 0x64:
				case 0x65:
				case 0x66:
				case 0x67:
					tmp8 = third_byte();
					PRINT("SUI %s, 0x%02X", str_r_names[op2 & 7], tmp8);
					PRINT_DESC("%s -= 0x%02X;", str_r_names[op2 & 7], tmp8);
					REG_BANK[op2 & 7] = sub8(REG_BANK[op2 & 7], tmp8, 0);
				break;
				case 0x70:		// SBI r, byte (Subtract immediate with borrow)
				case 0x71:
				case 0x72:
				case 0x73:
				case 0x74:
				case 0x75:
				case 0x76:
				case 0x77:
					tmp8 = third_byte();
					PRINT("SBI %s, 0x%02X", str_r_names[op2 & 7], tmp8);
					PRINT_DESC("%s -= (0x%02X + CY);", str_r_names[op2 & 7], tmp8);
					REG_BANK[op2 & 7] = sub8(REG_BANK[op2 & 7], tmp8, CY);
				break;
				default:
					printf("\r\nUnsupported instruction (0x%02X :: 0x%02X)\r\n", op, op2);
				break;
			};
		break;
		case 0x70: // Multiple instructions group
			GET_CODE_FLG;
			op2 = second_byte();
			switch (op2) {
				case 0x0E:	// SSPD word
					tmp8_1 = third_byte();
					tmp8 = fourth_byte();
					addr = (tmp8 << 8) | tmp8_1;
					PRINT("SSPD 0x%04X", addr);
					PRINT_DESC("@0x%04X = SP & 0xFF; @0x%04X = (SP >> 8) & 0xFF;", addr, addr+1);
				break;
				case 0x1E:	// SBCD word
					tmp8_1 = third_byte();
					tmp8 = fourth_byte();
					addr = (tmp8 << 8) | tmp8_1;
					PRINT("SBCD 0x%04X", addr);
					PRINT_DESC("@0x%04X = C; @0x%04X = B;", addr, addr+1);
				break;
				case 0x2E:	// SDED word
					tmp8_1 = third_byte();
					tmp8 = fourth_byte();
					addr = (tmp8 << 8) | tmp8_1;
					PRINT("SDED 0x%04X", addr);
					PRINT_DESC("@0x%04X = E; @0x%04X = D;", addr, addr+1);
				break;
				case 0x3E:	// SHLD word
					tmp8_1 = third_byte();
					tmp8 = fourth_byte();
					addr = (tmp8 << 8) | tmp8_1;
					PRINT("SHLD 0x%04X", addr);
					PRINT_DESC("@0x%04X = L; @0x%04X = H;", addr, addr+1);
				break;
				case 0x0F:	// LSPD word
					tmp8_1 = third_byte();
					tmp8 = fourth_byte();
					addr = (tmp8 << 8) | tmp8_1;
					tmp8 = read_mem(addr);
					tmp8_1 = read_mem(addr + 1);
					cpu.SP = (tmp8_1 << 8) | tmp8;
					PRINT("LSPD 0x%04X", addr);
					PRINT_DESC("SP = @0x%04X; SP |= (@0x%04X << 8);", addr, addr+1);
				break;
				case 0x1F:	// LBCD word
					tmp8_1 = third_byte();
					tmp8 = fourth_byte();
					addr = (tmp8 << 8) | tmp8_1;
					tmp8 = read_mem(addr);
					REG_C = tmp8;
					tmp8_1 = read_mem(addr + 1);
					REG_B = tmp8_1;
					PRINT("LBCD 0x%04X", addr);
					PRINT_DESC("B = @0x%04X; C = @0x%04X;", addr, addr + 1);
				break;
				case 0x2F:	// LDED word
					tmp8_1 = third_byte();
					tmp8 = fourth_byte();
					addr = (tmp8 << 8) | tmp8_1;
					tmp8 = read_mem(addr);
					REG_E = tmp8;
					tmp8_1 = read_mem(addr + 1);
					REG_D = tmp8_1;
					PRINT("LDED 0x%04X", addr);
					PRINT_DESC("E = @0x%04X; D = @0x%04X;", addr, addr + 1);
				break;
				case 0x3F:	// LHLD word
					tmp8_1 = third_byte();
					tmp8 = fourth_byte();
					addr = (tmp8 << 8) | tmp8_1;
					tmp8 = read_mem(addr);
					REG_L = tmp8;
					tmp8_1 = read_mem(addr + 1);
					REG_H = tmp8_1;
					PRINT("LHLD 0x%04X", addr);
					PRINT_DESC("L = @0x%04X; H = @0x%04X;", addr, addr + 1);
				break;
				case 0x99:	// ORAX rpa
				case 0x9A:
				case 0x9B:
				case 0x9C:
				case 0x9D:
				case 0x9E:
				case 0x9F:
					addr = get_rpa(op2 & 7);
					PRINT("ORAX @%s (0x%04X)", str_rpa_names[op2 & 7], addr);
					PRINT_DESC("A |= @0x%04X;", addr);
				break;
				case 0xC9:	// ONAX rpa
				case 0xCA:
				case 0xCB:
				case 0xCC:
				case 0xCD:
				case 0xCE:
				case 0xCF:
					condition_set = 1;
					addr = get_rpa(op2 & 7);
					PRINT("ONAX @%s (0x%04X)", str_rpa_names[op2 & 7], addr);
					PRINT_DESC("if(A & @0x%04X) {skip next}", addr);
				break;
				case 0xD9:	// OFFAX rpa
				case 0xDA:
				case 0xDB:
				case 0xDC:
				case 0xDD:
				case 0xDE:
				case 0xDF:
					condition_set = 1;
					addr = get_rpa(op2 & 7);
					PRINT("OFFAX @%s (0x%04X)", str_rpa_names[op2 & 7], addr);
					PRINT_DESC("if(0 == (A & @0x%04X)) {skip next}", addr);
				break;
				case 0xE9:	// NEAX rpa
				case 0xEA:
				case 0xEB:
				case 0xEC:
				case 0xED:
				case 0xEE:
				case 0xEF:
					condition_set = 1;
					addr = get_rpa(op2 & 7);
					PRINT("NEAX @%s (0x%04X)", str_rpa_names[op2 & 7], addr);
					PRINT_DESC("if(A != @0x%04X) {skip next}", addr);
				break;
				case 0xB9:	// LTAX rpa
				case 0xBA:
				case 0xBB:
				case 0xBC:
				case 0xBD:
				case 0xBE:
				case 0xBF:
					condition_set = 1;
					addr = get_rpa(op2 & 7);
					tmp8 = read_mem(addr);
					PRINT("LTAX @%s", str_rpa_names[op2 & 7]);
					PRINT_DESC("if(@%04X > A) {skip next}", addr);
				break;
				case 0xA9:	// GTAX rpa
				case 0xAA:
				case 0xAB:
				case 0xAC:
				case 0xAD:
				case 0xAE:
				case 0xAF:
					condition_set = 1;
					addr = get_rpa(op2 & 7);
					tmp8 = read_mem(addr);
					PRINT("GTAX @%s", str_rpa_names[op2 & 7]);
					PRINT_DESC("if(@%0x%04X < A) {skip next}", addr);
				break;
				case 0xF9:	// EQAX rpa
				case 0xFA:
				case 0xFB:
				case 0xFC:
				case 0xFD:
				case 0xFE:
				case 0xFF:
					condition_set = 1;
					addr = get_rpa(op2 & 7);
					PRINT("EQAX @%s (0x%04X)", str_rpa_names[op2 & 7], addr);
					PRINT_DESC("if(A == @0x%04X) {skip next}", addr);
				break;
				case 0x61:		// ESUB EA, r2
				case 0x62:
				case 0x63:
					PRINT("ESUB EA, %s", str_r_names[op2 & 3]);
					PRINT_DESC("if(EA < %s){CY = 1}; EA -= %s;", str_r_names[op2 & 3], str_r_names[op2 & 3]);
					REG_EA = sub16(REG_EA, REG_BANK[op2 & 3], 0);
				break;
				case 0x41:		// EADD EA, r2
				case 0x42:
				case 0x43:
					PRINT("EADD EA, %s", str_r_names[op2 & 3]);
					PRINT_DESC("EA += %s; // Sets CY if overflow", str_r_names[op2 & 3]);
					REG_EA = add16(REG_BANK[op2 & 3], REG_EA, 0);
				break;
				case 0x89:	// ANAX rpa
				case 0x8A:
				case 0x8B:
				case 0x8C:
				case 0x8D:
				case 0x8E:
				case 0x8F:
					addr = get_rpa(op2 & 7);
					tmp8 = read_mem(addr);
					PRINT("ANAX @%s", str_rpa_names[op2 & 7]);
					PRINT_DESC("A &= @0x%04X;", addr);
					REG_A &= tmp8;
				break;
				case 0x91:	// XRAX rpa
				case 0x92:
				case 0x93:
				case 0x94:
				case 0x95:
				case 0x96:
				case 0x97:
					addr = get_rpa(op2 & 7);
					tmp8 = read_mem(addr);
					PRINT("XRAX @%s", str_rpa_names[op2 & 7]);
					PRINT_DESC("A ^= @0x%04X;", addr);
					REG_A ^= tmp8;
				break;
				case 0xA1:	// ADDNCX rpa
				case 0xA2:
				case 0xA3:
				case 0xA4:
				case 0xA5:
				case 0xA6:
				case 0xA7:
					condition_set = 1;
					addr = get_rpa(op2 & 7);
					tmp8 = read_mem(addr);
					PRINT("ADDNCX @%s", str_rpa_names[op2 & 7]);
					PRINT_DESC("A += @0x%04X; if(A > 256) {skip next}", addr);
					REG_A = add8(REG_A, tmp8, 0);
				break;
				case 0xB1:	// SUBNBX rpa
				case 0xB2:
				case 0xB3:
				case 0xB4:
				case 0xB5:
				case 0xB6:
				case 0xB7:
					condition_set = 1;
					addr = get_rpa(op2 & 7);
					tmp8 = read_mem(addr);
					PRINT("SUBNBX @%s", str_rpa_names[op2 & 7]);
					PRINT_DESC("A -= @0x%04X; if(A >= 0) {skip next}", addr);
					REG_A = add8(REG_A, tmp8, 0);
				break;
				case 0xC1:	// ADDX rpa
				case 0xC2:
				case 0xC3:
				case 0xC4:
				case 0xC5:
				case 0xC6:
				case 0xC7:
					addr = get_rpa(op2 & 7);
					tmp8 = read_mem(addr);
					PRINT("ADDX @%s", str_rpa_names[op2 & 7]);
					PRINT_DESC("A += @0x%04X;", addr);
					REG_A = add8(REG_A, tmp8, 0);
				break;
				case 0xE1:	// SUBX rpa
				case 0xE2:
				case 0xE3:
				case 0xE4:
				case 0xE5:
				case 0xE6:
				case 0xE7:
					addr = get_rpa(op2 & 7);
					tmp8 = read_mem(addr);
					PRINT("SUBX @%s", str_rpa_names[op2 & 7]);
					PRINT_DESC("A -= @0x%04X;", addr);
					REG_A = sub8(REG_A, tmp8, 0);
				break;
				case 0xD1:	// ADCX rpa
				case 0xD2:
				case 0xD3:
				case 0xD4:
				case 0xD5:
				case 0xD6:
				case 0xD7:
					addr = get_rpa(op2 & 7);
					tmp8 = read_mem(addr);
					PRINT("ADCX @%s", str_rpa_names[op2 & 7]);
					PRINT_DESC("A += (@0x%04X + CY);", addr);
					REG_A = add8(REG_A, tmp8, CY);
				break;
				case 0xF1:	// SBBX rpa
				case 0xF2:
				case 0xF3:
				case 0xF4:
				case 0xF5:
				case 0xF6:
				case 0xF7:
					addr = get_rpa(op2 & 7);
					tmp8 = read_mem(addr);
					PRINT("SBBX @%s", str_rpa_names[op2 & 7]);
					PRINT_DESC("A -= (@0x%04X + CY);", addr);
					REG_A = sub8(REG_A, tmp8, CY);
				break;
				case 0x68:	// MOV r, word
				case 0x69:
				case 0x6A:
				case 0x6B:
				case 0x6C:
				case 0x6D:
				case 0x6E:
				case 0x6F:
					tmp8_1 = third_byte();
					tmp8 = fourth_byte();
					addr = (tmp8 << 8) | tmp8_1;
					tmp8 = read_mem(addr);
					PRINT("MOV %s, 0x%04X", str_r_names[op2 & 7], addr);
					PRINT_DESC("%s = @%04X;", str_r_names[op2 & 7], addr);
					REG_BANK[op2 & 7] = tmp8;
				break;
				case 0x78:	// MOV word, r
				case 0x79:
				case 0x7A:
				case 0x7B:
				case 0x7C:
				case 0x7D:
				case 0x7E:
				case 0x7F:
					tmp8_1 = third_byte();
					tmp8 = fourth_byte();
					addr = (tmp8 << 8) | tmp8_1;
					PRINT("MOV 0x%04X, %s", addr, str_r_names[op2 & 7]);
					PRINT_DESC("@%04X = %s;", addr, str_r_names[op2 & 7]);
				break;
				default:
					printf("\r\nUnsupported instruction (0x%02X :: 0x%02X)\r\n", op, op2);
				break;
			};
		break;

		case 0x18:	// MOV r1, A
		case 0x19:
		case 0x1A:
		case 0x1B:
		case 0x1C:
		case 0x1D:
		case 0x1E:
		case 0x1F:
			PRINT("MOV %s, A", str_r1_names[op & 7]);
			PRINT_DESC("%s = A;", str_r1_names[op & 7]);
			set_r1(op & 7, REG_A);
		break;

		case 0x00: // NOP
			PRINT("NOP");
			PRINT_DESC("No Operation");
		break;

		case 0x08:	// MOV A, r1
		case 0x09:
		case 0x0A:
		case 0x0B:
		case 0x0C:
		case 0x0D:
		case 0x0E:
		case 0x0F:
			REG_A = get_r1(op & 7);
			PRINT("MOV A, %s", str_r1_names[op & 7]);
			PRINT_DESC("A = %s;", str_r1_names[op & 7]);
		break;

		default:
			if( 0xC0 == (op & 0xC0) )		// JR word
			{
				uint16_t loc_pc = cpu.PC;
				if(table_instruction)
				{
					table_instruction = 0;
					disassem[cpu.PC].table_flg = TABLE_DATA;
				}

				if(op & 0x20){
					cpu.PC -= ((~op & 0x1F) + 1);
				} else {
					cpu.PC += (op & 0x1F);
				}
				int diff = cpu.PC - loc_pc;

				add_label(loc_pc - 1, cpu.PC, TYPE_JUMP);

				tmp32 = find_label(cpu.PC);
				if(tmp32){
					PRINT("JR %s", labels[tmp32 - 1].label);
					PRINT_DESC("goto %s;", labels[tmp32 - 1].label);
				} else {
					PRINT("JR @0x%04X", cpu.PC);
					PRINT_DESC("goto 0x%04X;", cpu.PC);
				}

				if(condition_set)
				{
					condition_set = 0;
					if( 0 == disassem[cpu.PC].state ) { push_registers(cpu.PC); }
					cpu.PC = loc_pc;
				}
			}
			else if( 0x80 == (op & 0xE0) )		// CALT byte
			{
				tmp16 = ((op & 0x1F) << 1) + 128;
				tmp8 = read_mem(tmp16);
				tmp8_1 = read_mem(tmp16 + 1);
				addr = (tmp8_1 << 8) | tmp8;

				add_label(cpu.PC - 1, addr, TYPE_FUNCTION);

				tmp32 = find_label(addr);
				if(tmp32){
					PRINT("CALT %s", labels[tmp32 - 1].label);
					PRINT_DESC("%s();", labels[tmp32 - 1].label);
				} else {
					PRINT("CALT @0x%04X", cpu.PC);
					PRINT_DESC("0x%04X();", cpu.PC);
				}

				push_registers(addr);
			} else {
				printf("\r\n%sUnknown opcode: 0x%02X at PC=0x%04X%s\n", RED, op, cpu.PC - 1, RESET);
#if 0
	for (int i = bin_offset; i < (rom_size + bin_offset); ++i)
	{
		int found = find_label(i);

		// If SOFTI instruction not used, do not print vector label
		if((0x60 == i) && !softi_used ) { found = 0; }
		
		// If label found print it
		if(found) { fprintf(f, "\r\n%s:\t\t; %u calls\r\n", labels[found - 1].label, labels[found - 1].count); }

		if( (1 == disassem[i].state) && (' ' != disassem[i].text[0]) ) {
			fprintf(f, "%s\r\n", disassem[i].text);
		} else if( (0 == disassem[i].state) ) {
			fprintf(f, "%04x %02x           .db %02Xh\r\n", i, rom[i] & 0xFF, rom[i] & 0xFF);
		}
	}
	exit(1);
#endif
			}
			break;
	}

	if(2 == condition_set) { condition_set = 0; }
	if(2 == table_instruction) { table_instruction = 0; }

	return cpu.PC;
}

static uint8_t fetch8(void) {
	disassem[cpu.PC].state = 1;
	return rom[cpu.PC++];
}

static uint8_t second_byte(void) {
	disassem[cpu.PC].state = 2;
	GET_CODE_FLG;
	uint8_t r = rom[cpu.PC++];
	rom_flg[cpu.PC] |= 2;
	snprintf(&raw_str[8], 9, "%02x       ", r);
	return r;
}

static uint8_t third_byte(void) {
	disassem[cpu.PC].state = 3;
	GET_CODE_FLG;
	uint8_t r = rom[cpu.PC++];
	rom_flg[cpu.PC] |= 4;
	snprintf(&raw_str[11], 6, "%02x    ", r);
	return r;
}

static uint8_t fourth_byte(void) {
	disassem[cpu.PC].state = 4;
	GET_CODE_FLG;
	uint8_t r = rom[cpu.PC++];
	rom_flg[cpu.PC] |= 8;
	snprintf(&raw_str[14], 3, "%02x ", r);
	return r;
}

static uint8_t read_mem(uint16_t addr) {
	GET_DATA_RD_FLG;
	return rom[addr];
}

static uint8_t add8(uint8_t a, uint8_t b, uint8_t c )
{
	uint16_t res = a + b + c;
	return res & 0xFF;
}

static uint16_t add16(uint16_t a, uint16_t b, uint16_t c )
{
	uint32_t res = a + b + c;
	return res & 0xFFFF;
}

static uint8_t sub8(uint8_t a, uint8_t b, uint8_t c )
{
	uint16_t res = a - b - c;
	return res & 0xFF;
}

static uint16_t sub16(uint16_t a, uint16_t b, uint16_t c )
{
	uint32_t res = a - b - c;
	return res & 0xFFFF;
}

static uint16_t get_rp(uint8_t rpa)
{
	uint16_t ret = 0;
	switch (rpa) {
		case 0: ret = cpu.SP; break;
		case 1: ret = GET_BC; break;
		case 2: ret = GET_DE; break;
		case 3: ret = GET_HL; break;
		default: printf(" Register pair error!!!\r\n"); break;
	}
	return ret;
}

static void set_rp(uint8_t rpa, uint16_t val)
{
	switch (rpa) {
		case 0: cpu.SP = val; break;
		case 1: SET_BC(val); break;
		case 2: SET_DE(val); break;
		case 3: SET_HL(val); break;
		default: printf(" Register pair error!!!\r\n"); break;
	}
}

static uint16_t get_rp3(uint8_t rpa)
{
	uint16_t ret = 0;
	switch (rpa) {
		case 1: ret = GET_BC; break;
		case 2: ret = GET_DE; break;
		case 3: ret = GET_HL; break;
		default: printf(" Register pair error!!!\r\n"); break;
	}
	return ret;
}

static void set_rp3(uint8_t rpa, uint16_t val)
{
	switch (rpa) {
		case 1: SET_BC(val); break;
		case 2: SET_DE(val); break;
		case 3: SET_HL(val); break;
		default: printf(" Register pair error!!!\r\n"); break;
	}
}

static uint16_t get_rpa(uint8_t rpa)
{
	uint16_t ret = 0;
	switch (rpa) {
		//case 0: ret = VA; break; // This one might not be in use!
		case 1: ret = GET_BC; break;
		case 2: ret = GET_DE; break;
		case 3: ret = GET_HL; break;
		case 4: ret = GET_DE; SET_DE(GET_DE + 1); break;
		case 5: ret = GET_HL; SET_HL(GET_HL + 1); break;
		case 6: ret = GET_DE; SET_DE(GET_DE - 1); break;
		case 7: ret = GET_HL; SET_HL(GET_HL - 1); break;
		default: printf(" Register pair error!!!\r\n"); break;
	}
	return ret;
}

static uint16_t get_rpa1(uint8_t rpa)
{
	uint16_t ret = 0;
	switch (rpa) {
		case 0: ret = GET_VA; break;
		case 1: ret = GET_BC; break;
		case 2: ret = GET_DE; break;
		case 3: ret = GET_HL; break;
		case 4: ret = REG_EA; break;
		default: printf(" Register pair error!!!\r\n"); break;
	}
	return ret;
}

static void set_rpa1(uint8_t rpa, uint16_t val)
{
	switch (rpa) {
		case 0: SET_VA(val); break;
		case 1: SET_BC(val); break;
		case 2: SET_DE(val); break;
		case 3: SET_HL(val); break;
		case 4: REG_EA = val; break;
		default: printf(" Register pair error!!!\r\n"); break;
	}
}

static uint16_t get_rpa2(uint8_t rpa, uint8_t byte)
{
	uint16_t ret = 0;
	switch (rpa) {
		case 1: ret = GET_BC; break;
		case 2: ret = GET_DE; break;
		case 3: ret = GET_HL; break;
		case 4: ret = GET_DE; SET_DE(GET_DE + 1); break;
		case 5: ret = GET_HL; SET_HL(GET_HL + 1); break;
		case 6: ret = GET_DE; SET_DE(GET_DE - 1); break;
		case 7: ret = GET_HL; SET_HL(GET_HL - 1); break;
		case 11: ret = GET_DE + byte; break;
		case 12: ret = GET_HL + REG_A; break;
		case 13: ret = GET_HL + REG_B; break;
		case 14: ret = GET_HL + REG_EA; break;
		case 15: ret = GET_HL + byte; break;
		default: printf(" Register pair error!!!\r\n"); break;
	}
	return ret;
}

static uint16_t get_rpa3(uint8_t rpa, uint8_t byte)
{
	uint16_t ret = 0;
	switch (rpa) {
		case 1: ret = GET_BC; break;
		case 2: ret = GET_DE; break;
		case 3: ret = GET_HL; break;
		case 4: ret = GET_DE; SET_DE(GET_DE + 2); break;
		case 5: ret = GET_HL; SET_HL(GET_HL + 2); break;
		case 6: ret = GET_DE; SET_DE(GET_DE - 2); break;
		case 7: ret = GET_HL; SET_HL(GET_HL - 2); break;
		case 11: ret = GET_DE + byte; break;
		case 12: ret = GET_HL + REG_A; break;
		case 13: ret = GET_HL + REG_B; break;
		case 14: ret = GET_HL + REG_EA; break;
		case 15: ret = GET_HL + byte; break;
		default: printf(" Register pair error!!!\r\n"); break;
	}
	return ret;
}

static uint8_t get_r1(uint8_t r1)
{
	uint8_t ret = 0;
	switch (r1) {
		case 0: ret = (REG_EA >> 8) & 0xFF; break;
		case 1: ret = REG_EA & 0xFF; break;
		case 2: ret = REG_B; break;
		case 3: ret = REG_C; break;
		case 4: ret = REG_D; break;
		case 5: ret = REG_E; break;
		case 6: ret = REG_H; break;
		case 7: ret = REG_L; break;
		default: printf(" Register r1 error!!!\r\n"); break;
	}
	return ret;
}

static void set_r1(uint8_t r1, uint8_t val)
{
	switch (r1) {
		case 0: REG_EA &= 0x00FF; REG_EA |= (val << 8); break;
		case 1: REG_EA &= 0xFF00; REG_EA |= val; break;
		case 2: REG_B = val; break;
		case 3: REG_C = val; break;
		case 4: REG_D = val; break;
		case 5: REG_E = val; break;
		case 6: REG_H = val; break;
		case 7: REG_L = val; break;
		default: printf(" Register r1 error!!!\r\n"); break;
	}
}

// EOF
