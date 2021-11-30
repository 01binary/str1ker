#include <pigpio.h>
#include <stdio.h>
#include <string.h>

const unsigned char SIGNATURE[] = { 'p', 'o', 't', 's' };

struct ARM {
	short shoulder;
	short upperarm;
	short forearm;
};

struct POTS {
	unsigned char sig[4];
	struct ARM arms[4];
};

int main() {
	int res = gpioInitialise();

	if (res < 0) {
		printf("Failed to initialize GPIO: 0x%x\n", res);
		return 1;
	}

	int ser = serOpen("/dev/ttyACM0", 9600, 0);

	if (ser < 0) {
		printf("Failed to open serial: 0x%x\n", ser);
		return 1;
	}

	int read = 0;
	char buf[128] = {0};
	struct POTS* pPots;

	while (true) {
		read += serRead(ser, buf + read, sizeof(POTS));
		if (read < sizeof(POTS)) continue;

		for (int offset = 0; offset < read; ++offset) {
			if (memcmp(buf + offset, SIGNATURE, sizeof(SIGNATURE)) == 0) {
				pPots = (struct POTS*)buf + offset;
				break;
			}
		}

		if (!pPots) continue;

		for (int n = 0, a = 0; n < 4; ++n, a += 3) {
			printf(
				"arm %d\n\tshoulder [A%d] %d\n\tupper [A%d] %d   \n\tfore [A%d] %d    \n",
				n,
				a,
				pPots->arms[n].shoulder,
				a + 1,
				pPots->arms[n].upperarm,
				a + 2,
				pPots->arms[n].forearm
			);
		}

		puts("----------");

		// Reset, OK to miss some samples for now
		read = 0;
		memset(buf, 0, sizeof(buf));
	}

	return 0;
}