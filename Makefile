CC=gcc
WARNINGS=-Wall -Wextra -Wformat=2 -Wno-unused-result

#CC=clang
#WARNINGS=-Weverything -Wno-padded -Wno-disabled-macro-expansion -Wno-cast-align

CFLAGS=${WARNINGS} -pthread -O2

all: ethtscal
