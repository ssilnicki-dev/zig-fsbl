This project is devoted to digging into the following topics:

1. ZIG programming
2. FSBL programming
3. muticore ARMv7 (with stm32mp15 as primary target)
4. ARM TrustZone, Secure and Non Secure modes

It should at least boot some OS kernel in non-secure mode, allowing it to communicate with secure mode code.

06/25: Migrated development from Ubuntu to a FreeBSD host and switched from the target-specific GCC-based STM toolchain debugger to the universal LLDB. Although there are several options to utilize LLDB from the VSCode (Emacs in my case) perspective — with the cppdbg type and an intermediate lldb-mi layer being one of them — a direct LLDB (DAP) connection to the target via OpenOCD seems more beneficial from my point of view.

To achieve this from FreeBSD host, LLVM must be recompiled from the ports tree with the libxml2 library installed beforehand. Otherwise, LLDB is configured without XML capability and lacks the ability to parse the register layout provided by OpenOCD to its DAP client.

FreeBSD porting note:
In my experience of porting the FreeBSD kernel onto the STM32MP1 SoC, I tried several approaches including reflashing the eMMC every time I needed to adjust printouts from the kernel context. This process consumed too much time, and eventually, I arrived at a more straightforward method: loading the kernel into RAM using the host debugger. One inevitable problem with this approach is the lack of sufficient SRAM to fit the entire kernel. The only viable solution is to load and run a small dedicated piece of code into SRAM, which initializes the minimal DDR-bound peripheral (in my case, the QSMP module from Ka-Ro Electronics with 512MB DDR), and then to load the kernel into DDR address space. Couple samples for progressing in this direction could be found under src/stm32mp1. The other drawback is the fact that in this case the kernel is running in secure mode of ARM's TEE, that is suboptimal for production environment. For me it worked well, given that this project was mainly dedicated to educational purposes.
