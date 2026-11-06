# Pruebas de la logica del medidor en la PC: make
CFLAGS = -std=c11 -O2 -Wall -Wextra -Werror -I../Core/Inc

test: test_medidor
	./test_medidor

test_medidor: test_medidor.c ../Core/Src/medidor.c ../Core/Inc/medidor.h
	$(CC) $(CFLAGS) test_medidor.c ../Core/Src/medidor.c -o $@

clean:
	rm -f test_medidor

.PHONY: test clean
