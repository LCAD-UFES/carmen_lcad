all:
	g++ -g -c -Wall wav_decoder.c wav_io.c wav2txt.c fftw3_util.c spectogram.c
	g++ -g -o wav2txt wav_decoder.o wav_io.o wav2txt.o fftw3_util.o spectogram.o -lm -lfftw3 `pkg-config --libs opencv`
teste_fft:
	g++ -g -c -Wall fft3_util.c teste_fft.c wav_decoder.cpp
	g++ -g -o teste_fft fft3_util.o teste_fft.o wav_decoder.o -lm -lfftw3
run:
	./wav2txt teste.wav spectogram.png
	# gnuplot script.gnu
run_teste_fft:
	./teste_fft
	gnuplot script_teste_fft.gnu
clean:
	rm -rf *.o wav2txt *.png *.dat wav2txt teste_fft
