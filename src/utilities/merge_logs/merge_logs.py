
import sys, decimal

if __name__ == "__main__":
	if len(sys.argv) < 4:
		print ''
		print 'Use python', sys.argv[0], '<master log> <slave log> <output log> [<Timestamp Correction | Default: 0.0>]'
		print ''
	else:
		master_log = sys.argv[1]
		slave_log = sys.argv[2]
		log_out = sys.argv[3]

		if len(sys.argv) >= 5:
			timestamp_correction = decimal.Decimal(sys.argv[4])
		else:
			timestamp_correction = 0.0

		f1 = open(master_log, 'r')
		f2 = open(slave_log, 'r')
		g = open(log_out, 'w')

		l1 = l2 = 0
		s1 = f1.readline()
		s2 = f2.readline()

		# escreve o cabecalho do log usando o log1
		while (s1[0:5] == 'PARAM') or (s1[0] == '#'):
			g.write(s1)
			s1 = f1.readline()
			l1 += 1

		# descarta o cabecalho do log2 
		while (s2[0:5] == 'PARAM') or (s2[0] == '#'):
			s2 = f2.readline()
			l2 += 1

		# timestamp das mensagens (Decimal has more precision than float in Python)
		splt_s1 = s1.rsplit(' ')
		splt_s2 = s2.rsplit(' ')
		t1 = decimal.Decimal(splt_s1[len(splt_s1) - 3])
		t2 = decimal.Decimal(splt_s2[len(splt_s2) - 3]) - decimal.Decimal(timestamp_correction)

		# descarta as mensagens do log2 com timestamp menor que o inicio do log1
		while (t2 < t1):
			s2 = f2.readline()
			splt_s2 = s2.rsplit(' ')
			t2 = decimal.Decimal(splt_s2[len(splt_s2) - 3]) - decimal.Decimal(timestamp_correction)
			l2 += 1

		first_t1 = t1
		first_relative_t1 = decimal.Decimal(splt_s1[len(splt_s1) - 1])

		# loop principal
		while (s1 != '') and (s2 != ''):

			splt_s1 = s1.rsplit(' ')
			splt_s2 = s2.rsplit(' ')
			t1 = decimal.Decimal(splt_s1[len(splt_s1) - 3])
			t2 = decimal.Decimal(splt_s2[len(splt_s2) - 3]) - decimal.Decimal(timestamp_correction)

			# checa se o valor do timestamp faz sentido
			# no log log_ponte3-do_log_ponte-20170220.txt tem uma mensagem do
			# laser ldmrs que esta incompleta e nao tem os campos finais (timestamp, host, etc.)
			if t1 < 1487500000:
				print 'Problema no log1 Linha ', l1
			if t2 < 1487500000:
				print 'Problema no log2 Linha ', l2

			if t1 < t2:
				g.write(s1)
				s1 = f1.readline()
				l1 += 1
			else:
				splt_s2 = splt_s2[:len(splt_s2)-1]
				splt_s2.append(str(t2 - first_t1 + first_relative_t1))
				# join the splitted line using space as separator
				line = ' '.join(splt_s2) 
				g.write(line + '\n')
				s2 = f2.readline()
				l2 += 1

		# se o log2 terminar antes que o log1, escreve as mensagens restantes do log1
		while s1 != '':
			g.write(s1)
			s1 = f1.readline()

		# idem para o caso do log1 terminar primeiro.
		while s2 != '':
			g.write(s2)
			s2 = f2.readline()

		f1.close()
		f2.close()
		g.close()



