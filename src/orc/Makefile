
all: motiondetectorc.c motiondetectorc.h

motiondetectorc.c : motiondetectorc.orc
	orcc --implementation -o motiondetectorc.c motiondetectorc.orc

motiondetectorc.h : motiondetectorc.orc
	orcc --header -o motiondetectorc.h motiondetectorc.orc