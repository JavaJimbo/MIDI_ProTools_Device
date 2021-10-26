#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/MIDI_ProTools.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/MIDI_ProTools.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=
else
COMPARISON_BUILD=
endif

ifdef SUB_IMAGE_ADDRESS

else
SUB_IMAGE_ADDRESS_COMMAND=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../usb_descriptors.c ../usb_device.c ../DelayPIC32.c ../main.c ../CRCmodbus.c ../I2C_4BUS_EEPROM_PIC32.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/1472/usb_descriptors.o ${OBJECTDIR}/_ext/1472/usb_device.o ${OBJECTDIR}/_ext/1472/DelayPIC32.o ${OBJECTDIR}/_ext/1472/main.o ${OBJECTDIR}/_ext/1472/CRCmodbus.o ${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/1472/usb_descriptors.o.d ${OBJECTDIR}/_ext/1472/usb_device.o.d ${OBJECTDIR}/_ext/1472/DelayPIC32.o.d ${OBJECTDIR}/_ext/1472/main.o.d ${OBJECTDIR}/_ext/1472/CRCmodbus.o.d ${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1472/usb_descriptors.o ${OBJECTDIR}/_ext/1472/usb_device.o ${OBJECTDIR}/_ext/1472/DelayPIC32.o ${OBJECTDIR}/_ext/1472/main.o ${OBJECTDIR}/_ext/1472/CRCmodbus.o ${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o

# Source Files
SOURCEFILES=../usb_descriptors.c ../usb_device.c ../DelayPIC32.c ../main.c ../CRCmodbus.c ../I2C_4BUS_EEPROM_PIC32.c



CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/MIDI_ProTools.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=32MX795F512L
MP_LINKER_FILE_OPTION=
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1472/usb_descriptors.o: ../usb_descriptors.c  .generated_files/6b408f487f93652d1601dadc514347cb9777eba1.flag .generated_files/264b14be9070dba4d4dfffc70b5034e62e04fc.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/usb_descriptors.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/usb_descriptors.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPIC32MX795F512L_PIM -I".." -I"../../../../Microchip/Include" -I"../../../../Microchip/Include/USB" -I"../../../../Microchip/USB" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/1472/usb_descriptors.o.d" -o ${OBJECTDIR}/_ext/1472/usb_descriptors.o ../usb_descriptors.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)    
	
${OBJECTDIR}/_ext/1472/usb_device.o: ../usb_device.c  .generated_files/970c9f2c58ac36b7f993c85c2987f70eb87818ef.flag .generated_files/264b14be9070dba4d4dfffc70b5034e62e04fc.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/usb_device.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/usb_device.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPIC32MX795F512L_PIM -I".." -I"../../../../Microchip/Include" -I"../../../../Microchip/Include/USB" -I"../../../../Microchip/USB" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/1472/usb_device.o.d" -o ${OBJECTDIR}/_ext/1472/usb_device.o ../usb_device.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)    
	
${OBJECTDIR}/_ext/1472/DelayPIC32.o: ../DelayPIC32.c  .generated_files/df0be2fa692abd4d922daec975e5fc7188f3c70d.flag .generated_files/264b14be9070dba4d4dfffc70b5034e62e04fc.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/DelayPIC32.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/DelayPIC32.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPIC32MX795F512L_PIM -I".." -I"../../../../Microchip/Include" -I"../../../../Microchip/Include/USB" -I"../../../../Microchip/USB" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/1472/DelayPIC32.o.d" -o ${OBJECTDIR}/_ext/1472/DelayPIC32.o ../DelayPIC32.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)    
	
${OBJECTDIR}/_ext/1472/main.o: ../main.c  .generated_files/41f5a02b20bfa52c86f671286e32cfbc85ed0134.flag .generated_files/264b14be9070dba4d4dfffc70b5034e62e04fc.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPIC32MX795F512L_PIM -I".." -I"../../../../Microchip/Include" -I"../../../../Microchip/Include/USB" -I"../../../../Microchip/USB" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/1472/main.o.d" -o ${OBJECTDIR}/_ext/1472/main.o ../main.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)    
	
${OBJECTDIR}/_ext/1472/CRCmodbus.o: ../CRCmodbus.c  .generated_files/53e2abd41cd7913064c0d45ce35a8ada51c54ee2.flag .generated_files/264b14be9070dba4d4dfffc70b5034e62e04fc.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/CRCmodbus.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/CRCmodbus.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPIC32MX795F512L_PIM -I".." -I"../../../../Microchip/Include" -I"../../../../Microchip/Include/USB" -I"../../../../Microchip/USB" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/1472/CRCmodbus.o.d" -o ${OBJECTDIR}/_ext/1472/CRCmodbus.o ../CRCmodbus.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)    
	
${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o: ../I2C_4BUS_EEPROM_PIC32.c  .generated_files/f37ce0708acd14b2a65190c120984dbd13f39716.flag .generated_files/264b14be9070dba4d4dfffc70b5034e62e04fc.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPIC32MX795F512L_PIM -I".." -I"../../../../Microchip/Include" -I"../../../../Microchip/Include/USB" -I"../../../../Microchip/USB" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o.d" -o ${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o ../I2C_4BUS_EEPROM_PIC32.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)    
	
else
${OBJECTDIR}/_ext/1472/usb_descriptors.o: ../usb_descriptors.c  .generated_files/76c72ecb72a7e2e4184de323fbe452ecd2cd3890.flag .generated_files/264b14be9070dba4d4dfffc70b5034e62e04fc.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/usb_descriptors.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/usb_descriptors.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPIC32MX795F512L_PIM -I".." -I"../../../../Microchip/Include" -I"../../../../Microchip/Include/USB" -I"../../../../Microchip/USB" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/1472/usb_descriptors.o.d" -o ${OBJECTDIR}/_ext/1472/usb_descriptors.o ../usb_descriptors.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)    
	
${OBJECTDIR}/_ext/1472/usb_device.o: ../usb_device.c  .generated_files/28a79c4ed0b8061f1cd3120f29fd16dd448f5c2.flag .generated_files/264b14be9070dba4d4dfffc70b5034e62e04fc.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/usb_device.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/usb_device.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPIC32MX795F512L_PIM -I".." -I"../../../../Microchip/Include" -I"../../../../Microchip/Include/USB" -I"../../../../Microchip/USB" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/1472/usb_device.o.d" -o ${OBJECTDIR}/_ext/1472/usb_device.o ../usb_device.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)    
	
${OBJECTDIR}/_ext/1472/DelayPIC32.o: ../DelayPIC32.c  .generated_files/1cdc4be072fb3536c26515ca424411db3e296320.flag .generated_files/264b14be9070dba4d4dfffc70b5034e62e04fc.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/DelayPIC32.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/DelayPIC32.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPIC32MX795F512L_PIM -I".." -I"../../../../Microchip/Include" -I"../../../../Microchip/Include/USB" -I"../../../../Microchip/USB" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/1472/DelayPIC32.o.d" -o ${OBJECTDIR}/_ext/1472/DelayPIC32.o ../DelayPIC32.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)    
	
${OBJECTDIR}/_ext/1472/main.o: ../main.c  .generated_files/b67071ba6fe5201d99f5c8b3adc39b9906632665.flag .generated_files/264b14be9070dba4d4dfffc70b5034e62e04fc.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/main.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPIC32MX795F512L_PIM -I".." -I"../../../../Microchip/Include" -I"../../../../Microchip/Include/USB" -I"../../../../Microchip/USB" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/1472/main.o.d" -o ${OBJECTDIR}/_ext/1472/main.o ../main.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)    
	
${OBJECTDIR}/_ext/1472/CRCmodbus.o: ../CRCmodbus.c  .generated_files/7397a354831fc91ee8c2dfb4276588aaf2d7e19.flag .generated_files/264b14be9070dba4d4dfffc70b5034e62e04fc.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/CRCmodbus.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/CRCmodbus.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPIC32MX795F512L_PIM -I".." -I"../../../../Microchip/Include" -I"../../../../Microchip/Include/USB" -I"../../../../Microchip/USB" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/1472/CRCmodbus.o.d" -o ${OBJECTDIR}/_ext/1472/CRCmodbus.o ../CRCmodbus.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)    
	
${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o: ../I2C_4BUS_EEPROM_PIC32.c  .generated_files/21424b2e13e01438a4b9728398bcef462c976b64.flag .generated_files/264b14be9070dba4d4dfffc70b5034e62e04fc.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o 
	${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -DPIC32MX795F512L_PIM -I".." -I"../../../../Microchip/Include" -I"../../../../Microchip/Include/USB" -I"../../../../Microchip/USB" -I"." -MP -MMD -MF "${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o.d" -o ${OBJECTDIR}/_ext/1472/I2C_4BUS_EEPROM_PIC32.o ../I2C_4BUS_EEPROM_PIC32.c    -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)    
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compileCPP
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/MIDI_ProTools.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE) -g -mdebugger -D__MPLAB_DEBUGGER_ICD3=1 -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/MIDI_ProTools.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)   -mreserve=data@0x0:0x1FC -mreserve=boot@0x1FC02000:0x1FC02FEF -mreserve=boot@0x1FC02000:0x1FC024FF  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_ICD3=1,--defsym=_min_heap_size=1000,-L"../../../../../mcc18/lib",-L".",-Map="${DISTDIR}/_MIDI_Demo.X.${IMAGE_TYPE}.map" 
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/MIDI_ProTools.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/MIDI_ProTools.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -DXPRJ_default=$(CND_CONF)    $(COMPARISON_BUILD)  -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=_min_heap_size=1000,-L"../../../../../mcc18/lib",-L".",-Map="${DISTDIR}/_MIDI_Demo.X.${IMAGE_TYPE}.map" 
	${MP_CC_DIR}\\xc32-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/MIDI_ProTools.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} 
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
