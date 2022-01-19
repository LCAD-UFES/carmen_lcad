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

ifeq (,$(wildcard $(CARMEN_HOME)/bin/atego1730_mosaic/semi_reboque_basculante.obj))
	$(info )
	$(info **************** **********************************************************************************************)
	$(info *	Dowloading 3D model semi_reboque_basculante.obj (The semi_trailer))
	$(info ***************************************************************************************************************)
	$(info )
	$(CARMEN_HOME)/bin/gdown.pl https://drive.google.com/file/d/1jyMh1QqvySpWtbfylwuFnsA2WqRUVtBd/view?usp=sharing $(CARMEN_HOME)/bin/atego1730_mosaic/semi_reboque_basculante.obj
else

	$(info * The semi_trailer 3D model already exists *)
endif

