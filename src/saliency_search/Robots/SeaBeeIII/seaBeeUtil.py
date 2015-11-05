#!/usr/bin/python

import os, sys

execAll = sys.argv.count("--all")

if sys.argv.count("--clear-pf-state") > 0 or execAll:
	#delete ~/.seabee/ParticleFilterState.txt
	os.system("rm /home/uscr/.seabee/ParticleFilterState.txt")
	print "\nCleared ParticleFilter state.\n"
	
if sys.argv.count("--clear-pf-params") > 0 or execAll:
	#delete ~/.seabee/ParticleFilterParams.txt
	os.system("rm /home/uscr/.seabee/ParticleFilterParams.txt")
	print "\nCleared ParticleFilter parameters.\n"
