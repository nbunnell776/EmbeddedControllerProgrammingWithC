#include <stdio.h>

int main(void)
{
	__uint32_t flashSize = 0x0400;

	__uint32_t uidArr[3];
	uidArr[0] =  0x0000000000240040;
	uidArr[1] =  0x00000000464b5013;
	uidArr[2] =  0x0000000020333253;

	char* ptr = &flashSize;

	printf("Flash size (KB): ");

	for (int i = 0; i < sizeof(int); ++i)
		{
			printf("%hx", *(ptr++));
		}

	printf("\n");

	ptr = &uidArr[0];

	printf("Device ID: 0x");

	for (int i = 0; i < ((3 * sizeof(int))/sizeof(char)); ++i)
		{
			//printf("0x%2hx\n", *(ptr++));
			printf("%02hx", *(ptr + i));
		}

	printf("\n");

	return 0;
}
