.PHONY: all uart uart1

all : clean uart uart1 uart_coe uart1_coe

uart: 
	cd ./uart/ && make all

uart1: 
	cd ./uart1/ && make all

uart_coe:
	../../bin2coe.py 32 < ./uart/user_uart.bin > ./uart/user_uart.coe

uart1_coe:
	../../bin2coe.py 32 < ./uart1/user_uart.bin > ./uart1/user_uart.coe

clean:
	rm -rf ./uart/*.img ./uart/*.dump ./uart/*.bin ./uart/*.elf ./uart1/*.img ./uart1/*.dump ./uart1/*.bin ./uart1/*.elf
