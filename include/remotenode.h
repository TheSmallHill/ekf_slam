#pragma once
#include <stdint.h>
#include <iostream>
#include <string.h>
#include <unistd.h>
#include <list>
#include <iomanip>

#include <xbeep.h>

class remotenode {
	public:
		explicit remotenode(std::string name, uint16_t addr16, uint64_t addr64):
			name(name),
			addr16(addr16),
			addr64(addr64) { };
		explicit remotenode(std::vector<unsigned char> nd_payload);

		std::string getName(void)     const { return name;                        }
		uint16_t    getAddr16(void)   const { return addr16;                      }
		uint64_t    getAddr64(void)   const { return addr64;                      }
		uint32_t    getAddr64Hi(void) const { return (addr64 >> 32) & 0xFFFFFFFF; }
		uint32_t    getAddr64Lo(void) const { return addr64 & 0xFFFFFFFF;         }

	private:
		std::string name;
		uint16_t addr16;
		uint64_t addr64;
};
