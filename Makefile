ledmatrix:
	gcc -o ledmatrix main.c -l wiringPi -Ofast
clean:
	rm ledmatrix
