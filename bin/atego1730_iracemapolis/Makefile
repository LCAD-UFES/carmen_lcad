download :
ifeq (,$(wildcard $(CARMEN_HOME)/bin/atego1730_suzano/modelo_atego1730-reescalado.obj))
	$(info )
	$(info **************** **********************************************************************************************)
	$(info *	Dowloading 3D model Mercedes ATEGO 1730 without the trailer)
	$(info * Dont worry if the .mtl returns error, the gdown doesn't work properly wtth small files *)
	$(info * Check the file and if necessary, download it direct from the link in this makefie *)
	$(info ***************************************************************************************************************)
	$(info )
	$(CARMEN_HOME)/bin/gdown.pl https://drive.google.com/file/d/1wtwRshrZmP-I-qfPVZCGYSEl8W3panlU/view?usp=sharing $(CARMEN_HOME)/bin/atego1730_suzano/modelo_atego1730-reescalado.obj
else

	$(info * The modelo_atego1730-reescalado.obj 3D model already exists *)
endif

ifeq (,$(wildcard $(CARMEN_HOME)/bin/atego1730_iracemapolis/semi_reboque_graneleiro.obj))
	$(info )
	$(info **************** **********************************************************************************************)
	$(info *	Dowloading 3D model semi_reboque_graneleiro.obj (The semi_trailer))
	$(info ***************************************************************************************************************)
	$(info )
	$(CARMEN_HOME)/bin/gdown.pl https://drive.google.com/file/d/16Z3DbrEDuOOn3X0CFvHzTChMab6GLD33/view?usp=sharing $(CARMEN_HOME)/bin/atego1730_iracemapolis/semi_reboque_graneleiro.obj
else

	$(info * The semi_trailer 3D model already exists *)
endif

