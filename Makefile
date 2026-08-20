CC      ?= cc
CFLAGS  ?= -std=c99 -Wall -Wextra -Wconversion -O1
# Catch the signed-overflow / bad-shift undefined behavior the controller
# can hit. Plain `undefined` covers signed overflow and oversized shifts;
# it deliberately excludes defined unsigned wraparound (used by the test
# suite's LCG). -fsanitize-recover reports every site instead of aborting
# on the first, so one run shows all of them.
SANFLAGS ?= -fsanitize=undefined -fsanitize-recover=undefined

.PHONY: all test test-sanitize demo clean

all: test

test: test_zheight
	./test_zheight

# Same tests under UBSan: signed overflow and oversized shifts abort.
test-sanitize: zheight.c test_zheight.c zheight.h test_harness.h
	$(CC) $(CFLAGS) $(SANFLAGS) -o test_zheight_san zheight.c test_zheight.c
	./test_zheight_san

test_zheight: zheight.c test_zheight.c zheight.h test_harness.h
	$(CC) $(CFLAGS) -o $@ zheight.c test_zheight.c

demo: zheight.c zheight_demo.c zheight.h
	$(CC) $(CFLAGS) -o zheight_demo zheight.c zheight_demo.c
	./zheight_demo

clean:
	rm -f test_zheight test_zheight_san zheight_demo
