
all: motiondetectorc.c motiondetectorc.h transformorc.c transformorc.h

motiondetectorc.c : motiondetectorc.orc
	orcc --implementation -o motiondetectorc.c motiondetectorc.orc

motiondetectorc.h : motiondetectorc.orc
	orcc --header -o motiondetectorc.h motiondetectorc.orc

transformorc.c : transformorc.orc
	orcc --implementation -o transformorc.c transformorc.orc

transformorc.h : transformorc.orc
	orcc --header -o transformorc.h transformorc.orc