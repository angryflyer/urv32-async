# find  -name  ".*v"   > flist.f
# INCLUDE=/work/home/zhaowei/work/software/urv32/rtl/com
# FSDB=FSDB_INST
# FSDB=FSDB_USOC
# TB=COREMARK
# TB=DHRYSTONE
# TB=INST
# TB=TEST
# BOOT=ROM //boot from rom
# BOOT=RAM //boot from ram
# SIM_FREQ=32
# SIM_TIME=20   // 20ms
# SIM_PERIOD=15.625 // 15.625ns 32MHz clk=~clk

INC_DIR=${PROJ_ROOT}/rtl/com

ifndef BOOT
	BOOT = ROM
endif

vcs		:
		vcs 	\
		-timescale=1ns/1ns \
		+define+$(TB)=1 \
		+define+$(FSDB)=1 \
		+define+$(BOOT)=1 \
		+define+SIM_FREQ=32 \
		+define+SIM_TIME=100 \
		+define+SIM_PERIOD=15.625 \
		+incdir+${INC_DIR} \
		-F flist.f \
		-top mem2ahb_tb \
		-sverilog \
		-v2k_generate \
		-kdb \
		-fsdb -full64 -R +vc+list -debug_access+all \
		+backdoor_load_image=$(IMAGE_PATH) \
		| tee vcs_$(TB).log
verdi 	:
		Verdi -SX -sv -F flist.f -ssf mem2ahb_tb.fsdb -top mem2ahb_tb -sswr signal.rc &

dhrystone: 
	cd ../../../firmware/dhrystone/ && make

coremark: 
	cd ../../../firmware/coremark/ && make

uart: 
	cd ../../../firmware/peri/uart/ && make

clean	: clean_dhrystone clean_coremark clean_uart
		rm -rf *.log simv *.daidir csrc *.key DVEfiles *.vpd *.conf novas.rc *.fsdb verdiLog 

clean_dhrystone: 
	cd ../../../firmware/dhrystone/ && make clean

clean_coremark: 
	cd ../../../firmware/coremark/ && make clean

clean_uart: 
	cd ../../../firmware/peri/uart/ && make clean
