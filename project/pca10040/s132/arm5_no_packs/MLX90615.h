#ifndef MLX90615_H
#define MLX90615_H


//MLX90614 constants
#define SA				0x5B	// Slave address
#define DEFAULT_SA		0x5A	// Default Slave address
//#define RAM_Access		0x00	// RAM access command
#define RAM_Access		0x10	// RAM access command

#define EEPROM_Access	0x20	// EEPROM access command
#define RAM_Tobj1		0x07	// To1 address in the eeprom

float mlx90615_read(void);
bool mlx90615_register_read(unsigned char register_address, unsigned char * destination, unsigned char number_of_bytes);

#endif /* MLX90615_H */

