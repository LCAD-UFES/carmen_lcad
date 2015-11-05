#!/usr/bin/env python

try:
  import gtk
except:
  print >> sys.stderr, "You need to install the python gtk bindings"
  sys.exit(1)

# import vte
try:
	import vte
except:
	error = gtk.MessageDialog (None, gtk.DIALOG_MODAL, gtk.MESSAGE_ERROR, gtk.BUTTONS_OK,
			'You need to install python bindings for libvte')
	error.run()
	sys.exit (1)

import time

v = {}
hosts = {}
hosts['bx1'] = '192.168.0.64'
hosts['bx2'] = '192.168.0.65'
hosts['bx3'] = '192.168.0.66'
hosts['bx4'] = '192.168.0.67'
hosts['bx5'] = '192.168.0.68'
hosts['bx6'] = '192.168.0.69'
hosts['bx7'] = '192.168.0.70'
hosts['bx8'] = '192.168.0.71'

#GistSal list
#gslist = {'M':'bx4','M2':'bx4','W61':'bx6'}
gslist ={}
gslist['4GS'] = 'bx4'
gslist['4LM'] = 'bx4'
gslist['5W0'] = 'bx5'
gslist['5W1'] = 'bx5'
gslist['6W2'] = 'bx6'
gslist['6W3'] = 'bx6'
gslist['8W4'] = 'bx8'
gslist['8W5'] = 'bx8'
gslist['3BP'] = 'bx3'
gslist['7NV'] = 'bx7'


class beoTabs(gtk.Notebook):

	def __init__(self):
		gtk.Notebook.__init__(self)
		self.set_tab_pos(gtk.POS_TOP)
		self.set_property('homogeneous',True)
		self.create_tabs()
		self.set_focus_tab()

	def set_focus_tab(self):
		current_frame = self.get_nth_page(0)
		id = current_frame.get_label()
		self.close_button[id].show()

	def close_tab(self,widget,child)	:
		page = 	self.page_num(child)
		id = child.get_label()
		if page != -1:
			self.remove_page(page)
			del v[id]
			del self.close_button[id]

			if self.get_n_pages() == 1:
				self.set_show_tabs(False)
	def close_current_tab(self,widget) :
		self.close_current_tab()
	def close_current_tab(self) :
		page = self.get_current_page()
		child = self.get_nth_page(page)
		id = child.get_label()
		if page != -1:
			self.remove_page(page)
			del v[id]
			del self.close_button[id]

			if self.get_n_pages() == 1:
				self.set_show_tabs(False)

	def close_all_tabs(self):
		for i in range(self.get_n_pages()):
			self.close_current_tab()



	def insert_tab(self,host):
		self.insert_tab_with_name(host,host)
	def insert_tab_with_name(self,host,name):
		print "Try insert label["+name+"] host = "+host
		try:
			v[host]
		except KeyError:
			#There is no terminal yet
			v[host] = vte.Terminal()
		 	v[host].connect ("child-exited", self.close_current_tab)
		 	v[host].fork_command()
			v[host].set_scrollback_lines(500000)#infinite scrollback

			scrollbar = gtk.VScrollbar()
			scrollbar.set_adjustment(v[host].get_adjustment())
			scrollbox = gtk.HBox(False,0)
			scrollbox.pack_start(v[host],True,True)
			scrollbox.pack_start(scrollbar,False,False)
			scrollbox.show_all()
			
			frame = gtk.Frame(host)
			frame.set_border_width(5)
			frame.add(scrollbox)

			self.label[host] = gtk.Label(name+'  ')
			self.label[host].show()
			self.label2[host] = gtk.Label('')
			self.label2[host].hide()
			#Add Close Button
			image = gtk.Image()
			image.set_from_stock(gtk.STOCK_CLOSE,gtk.ICON_SIZE_MENU)
			self.close_button[host] = gtk.Button()
			self.close_button[host].set_image(image)
			self.close_button[host].set_relief(gtk.RELIEF_NONE)
			self.close_button[host].connect("clicked",self.close_tab,frame)
			self.close_button[host].hide

			#Add Check Button
			self.check_button[host] = gtk.CheckButton()
			self.check_button[host].set_relief(gtk.RELIEF_NONE)
			self.check_button[host].set_active(True)
			self.check_button[host].show_all()

			hbox = gtk.HBox(False,0)
			hbox.pack_start(self.check_button[host],True,True)
			hbox.pack_start(self.label[host],True,True)
			hbox.pack_start(self.label2[host],True,True)
			hbox.pack_end(self.close_button[host],False,False)
			#hbox.show_all()
			
			self.append_page(frame,hbox)
			# Make Tab dragable
			self.set_tab_reorderable(frame,True)

			print 'Create New terminal' + host
			if self.get_n_pages() > 1:
				self.set_show_tabs(True)
			self.show_all()
			if(host.find('-') != -1):
				id,n  = host.split('-')
				cmd = 'ssh -CXY ' + id +'@'+hosts[id]
			else:	
				cmd = 'ssh -CXY ' + host +'@'+hosts[host]
			v[host].feed_child(cmd+'\n\r')
		else:
			#Terminal Already Exist
			print 'Terminal Exist Rename it'
			print 'length of ' + host+' is '+str(len(host))
			if(host.find('-') != -1):
				id,n  = host.split('-')
				newid = id+'-'+str(int(n)+1)
				self.insert_tab_with_name(newid,name)
			else:	
				self.insert_tab_with_name(host+'-1',name)
			
	def create_tabs(self):

		#Tabs
		self.close_button = {}
		self.check_button = {}
		self.label = {}
		#The label only show when close button is hide
		self.label2 = {}
		for i in range(8):

			buf = "bx%d" % (i+1)
			self.insert_tab(buf)

		#Add New Tab Button	
		#image = gtk.Image()
		#image.set_from_stock(gtk.STOCK_ADD,gtk.ICON_SIZE_MENU)
		#add_tab_button = gtk.Button()
		#add_tab_button.set_image(image)
		#add_tab_button.set_relief(gtk.RELIEF_NONE)
		#add_tab_button.show()
		#self.append_page(add_tab_button,add_tab_button)



class beoTerminal():

	def __init__(self):
		#Button
		btn_connect = gtk.Button("Connect All (x)")
		btn_connect.connect("clicked",self.do_connect,"Connect")
		btn_ctrlc = gtk.Button("Send Ctrl-C to all (c)")
		btn_ctrlc.connect("clicked",self.do_ctrlc,"Ctrl-C")
		btn_rlc = gtk.Button("Run Last Command to all (v)")
		btn_rlc.connect("clicked",self.do_rlc,"Run Last Command")
		btn_svn = gtk.Button("SVN update  (s)")
		btn_svn.connect("clicked",self.do_svn,"svn up")
		btn_box = gtk.HBox(False,0)
		btn_box.pack_start(btn_connect,True,True,0)
		btn_box.pack_start(btn_ctrlc,True,True,0)
		btn_box.pack_start(btn_rlc,True,True,0)
		btn_box.pack_start(btn_svn,True,True,0)

		# Menu
		menu_bar = gtk.MenuBar()
	
		#File Menu		
		menu_quit = gtk.MenuItem("Quit")
		menu_close_tab = gtk.MenuItem("Close Tab")
		menu_file = gtk.Menu()
		menu_file.append(gtk.SeparatorMenuItem())
		menu_file.append(menu_close_tab)
		menu_file.append(menu_quit)
		menu_root_file = gtk.MenuItem("_File")
		menu_root_file.set_submenu(menu_file)
		menu_bar.append(menu_root_file)
		menu_close_tab.connect("activate",self.on_close_current_tab)
		menu_quit.connect("activate", lambda term: gtk.main_quit())
	
		#Edit Menu
		menu_edit_copy = gtk.MenuItem("Copy")
		menu_edit_paste = gtk.MenuItem("Paste")
		menu_edit_selectall = gtk.MenuItem("Select All")
		menu_edit = gtk.Menu()
		menu_edit.append(menu_edit_copy)
		menu_edit.append(menu_edit_paste)
		menu_edit.append(gtk.SeparatorMenuItem())
		menu_edit.append(menu_edit_selectall)

		menu_root_edit = gtk.MenuItem("_Edit")
		menu_root_edit.set_submenu(menu_edit)
		menu_bar.append(menu_root_edit)

		menu_edit_copy.connect("activate",self.on_edit_copy)
		menu_edit_paste.connect("activate",self.on_edit_paste)
		#View Menu
		menu_view_toolbar = gtk.MenuItem("Toolbar")
		menu_view = gtk.Menu()
		menu_view.append(menu_view_toolbar)
		menu_root_view = gtk.MenuItem("V_iew")
		menu_root_view.set_submenu(menu_view)
		menu_bar.append(menu_root_view)
	
		#Tabs Menu
		menu_tabs_rename = gtk.MenuItem("Rename Tab")
		menu_tabs_previous = gtk.MenuItem("Previous Tab")
		menu_tabs_next= gtk.MenuItem("Next Tab")
		menu_tabs_left= gtk.MenuItem("Move Tab to the Left")
		menu_tabs_right= gtk.MenuItem("Move Tab to the Right")
		menu_tabs_lock= gtk.CheckMenuItem("Lock Tabs")
		menu_tabs_select = gtk.CheckMenuItem("Select All Tabs")
		menu_tabs_select.set_active(True)
		menu_tabs = gtk.Menu()
		menu_tabs.append(menu_tabs_rename)
		menu_tabs.append(gtk.SeparatorMenuItem())
		menu_tabs.append(menu_tabs_previous)
		menu_tabs.append(menu_tabs_next)
		menu_tabs.append(gtk.SeparatorMenuItem())
		menu_tabs.append(menu_tabs_left)
		menu_tabs.append(menu_tabs_right)
		menu_tabs.append(gtk.SeparatorMenuItem())
		menu_tabs.append(menu_tabs_lock)
		menu_tabs.append(menu_tabs_select)
		menu_root_tabs = gtk.MenuItem("_Tabs")
		menu_root_tabs.set_submenu(menu_tabs)
		menu_bar.append(menu_root_tabs)
	
		menu_tabs_rename.connect("activate",self.on_rename_tab)
		menu_tabs_previous.connect("activate",self.on_pre_tab)
		menu_tabs_next.connect("activate",self.on_next_tab)
		menu_tabs_left.connect("activate",self.on_left_tab)
		menu_tabs_right.connect("activate",self.on_right_tab)
		menu_tabs_lock.connect("activate",self.on_lock_tab)
		menu_tabs_select.connect("activate",self.on_select_tab)
	
		#Hosts Menu
		menu_hosts = gtk.Menu()
		menu_hosts_load = gtk.MenuItem("Load Hosts")
		menu_hosts_save = gtk.MenuItem("Save Hosts")
		menu_hosts_gistSal = gtk.MenuItem("GistSal Hosts")
		menu_hosts_gistSal.connect("activate",self.on_host_open_load,gslist)
		menu_hosts_multiple = gtk.MenuItem("Open Multiple Tabs")

		menu_hosts.append(menu_hosts_load)
		menu_hosts.append(menu_hosts_save)
		menu_hosts.append(gtk.SeparatorMenuItem())
		menu_hosts.append(menu_hosts_gistSal)
		menu_hosts.append(gtk.SeparatorMenuItem())


		menu_hosts_multiple_sub = gtk.Menu()
		menu_hosts_multiple.set_submenu(menu_hosts_multiple_sub)
		menu_hosts_open = gtk.MenuItem("Open Selected")
		menu_hosts_open.connect("activate",self.on_host_open_all)
		menu_hosts_select = gtk.CheckMenuItem("Select All")
		menu_hosts_select.set_active(True)
		menu_hosts_select.connect("activate",self.on_host_select)
		menu_hosts_multiple_sub.append(menu_hosts_open)
		menu_hosts_multiple_sub.append(gtk.SeparatorMenuItem())
		menu_hosts_multiple_sub.append(menu_hosts_select)
		menu_hosts_multiple_sub.append(gtk.SeparatorMenuItem())
		menu_hosts_multiple_sub.append(gtk.SeparatorMenuItem())

		menu_hosts.append(menu_hosts_multiple)
		self.menu_hosts_ = {}
		self.menu_hosts_c = {}
		for i in range(8):
			buf = "bx%d" % (i+1)
			self.menu_hosts_c[buf] = gtk.CheckMenuItem(buf)
			self.menu_hosts_c[buf].set_active(True)
			menu_hosts_multiple_sub.append(self.menu_hosts_c[buf])
		for i in range(8):
			buf = "bx%d" % (i+1)
			self.menu_hosts_[buf] = gtk.MenuItem(buf)
			self.menu_hosts_[buf].connect("activate",self.on_host_open,buf)
			menu_hosts.append(self.menu_hosts_[buf])


		menu_root_hosts = gtk.MenuItem("H_osts")
		menu_root_hosts.set_submenu(menu_hosts)
		menu_bar.append(menu_root_hosts)

		
		#Help Menu
		menu_help_about = gtk.MenuItem("About")

		menu_help = gtk.Menu()
		menu_help.append(menu_help_about)

		menu_root_help = gtk.MenuItem("_Help")
		menu_root_help.set_submenu(menu_help)
		menu_bar.append(menu_root_help)


		#Accelerator for hotkey
		agr = gtk.AccelGroup()
		#window.add_accel_group(agr)

		#Tabs
		self.tabs = beoTabs()		
		self.tabs.connect("switch-page",self.on_switch_page)	

		#Tabs Hotkey

		#Close Window
		key, mod = gtk.accelerator_parse("<Control>Q")
		menu_quit.add_accelerator("activate",agr,key,mod,gtk.ACCEL_VISIBLE)

		#Close Current Tab
		key, mod = gtk.accelerator_parse("<Control>W")
		menu_close_tab.add_accelerator("activate",agr,key,mod,gtk.ACCEL_VISIBLE)
		key, mod = gtk.accelerator_parse("<Control>Delete")
		menu_close_tab.add_accelerator("activate",agr,key,mod,gtk.ACCEL_VISIBLE)		

		#Copy
		key, mod = gtk.accelerator_parse("<Control><Shift>C")
		menu_edit_copy.add_accelerator("activate",agr,key,mod,gtk.ACCEL_VISIBLE)

		#Paste
		key, mod = gtk.accelerator_parse("<Control><Shift>V")
		menu_edit_paste.add_accelerator("activate",agr,key,mod,gtk.ACCEL_VISIBLE)

		#Select All
		key, mod = gtk.accelerator_parse("<Control>A")
		menu_edit_selectall.add_accelerator("activate",agr,key,mod,gtk.ACCEL_VISIBLE)

		#Rename Tab
		key, mod = gtk.accelerator_parse("<Control>R")
		menu_tabs_rename.add_accelerator("activate",agr,key,mod,gtk.ACCEL_VISIBLE)
		#Next Tab
		key, mod = gtk.accelerator_parse("<Control>Page_Down")
		menu_tabs_next.add_accelerator("activate",agr,key,mod,gtk.ACCEL_VISIBLE)
		key, mod = gtk.accelerator_parse("<Shift>Right")
		menu_tabs_next.add_accelerator("activate",agr,key,mod,gtk.ACCEL_VISIBLE)		
		#Previous Tab		
		key, mod = gtk.accelerator_parse("<Control>Page_Up")
		menu_tabs_previous.add_accelerator("activate",agr,key,mod,gtk.ACCEL_VISIBLE)
		key, mod = gtk.accelerator_parse("<Shift>Left")
		menu_tabs_previous.add_accelerator("activate",agr,key,mod,gtk.ACCEL_VISIBLE)
		#Move tab to the left
		key, mod = gtk.accelerator_parse("<Shift><Control>Page_Up")
		menu_tabs_left.add_accelerator("activate",agr,key,mod,gtk.ACCEL_VISIBLE)
		#Move tab to the right
		key, mod = gtk.accelerator_parse("<Shift><Control>Page_Down")
		menu_tabs_right.add_accelerator("activate",agr,key,mod,gtk.ACCEL_VISIBLE)

		#Button Hotkey
		key, mod = gtk.accelerator_parse("<Alt>x")
		btn_connect.add_accelerator("activate",agr,key,mod,gtk.ACCEL_VISIBLE)
		key, mod = gtk.accelerator_parse("<Alt>c")
		btn_ctrlc.add_accelerator("activate",agr,key,mod,gtk.ACCEL_VISIBLE)
		key, mod = gtk.accelerator_parse("<Alt>v")
		btn_rlc.add_accelerator("activate",agr,key,mod,gtk.ACCEL_VISIBLE)
		

		box = gtk.VBox(False,0)
		box.pack_start(menu_bar,True,True,0)
		box.pack_start(btn_box,True,True,0)
		box.pack_start(self.tabs,True,True,0)

	 	window = gtk.Window()
		window.add(box)
	 	window.connect('delete-event', lambda window, event: gtk.main_quit())
	 	window.connect('key-press-event', self.key_press_event)
		window.add_accel_group(agr)
	 	window.show_all()
	def get_current_child(self):
		nth =  self.tabs.get_current_page()
		return  self.tabs.get_nth_page(nth)

	def get_current_ID(self):
		return self.get_current_child().get_label()

	def cmd_all_checked_tabs(self,cmd):
		num = self.tabs.get_n_pages()
		for i in range(int(num)):
			frame = self.tabs.get_nth_page(i)
			id = frame.get_label()
			if(self.tabs.check_button[id].get_active()):	
				v[id].feed_child(cmd+'\n\r')
		print cmd
		
	def cmd_all_tabs(self,cmd):
		num = self.tabs.get_n_pages()
		for i in range(int(num)):
			frame = self.tabs.get_nth_page(i)
			id = frame.get_label()
			v[id].feed_child(cmd+'\n\r')
		print cmd
	def do_connect(self,widget,event):
		num = self.tabs.get_n_pages()
		for i in range(int(num)):
			frame = self.tabs.get_nth_page(i)
			id = frame.get_label()
			#Handle bx1 or bx1-1 type of name
			if(id.find('-')):
				newid = id.split('-')
				cmd = 'ssh -CXY ' + newid[0] +'@'+hosts[newid[0]]
			else:	
				cmd = 'ssh -CXY ' + id +'@'+hosts[id]
			if(self.tabs.check_button[id].get_active()):	
				v[id].feed_child(cmd+'\n\r')
		print cmd
	def do_ctrlc(self,widget,event):
		#Send Ctrl-C signal 0x03
		cmd = '\x03'
		self.cmd_all_tabs(cmd)
	def do_rlc(self,widget,event):
		cmd = '!!\n\r'
		self.cmd_all_checked_tabs(cmd)

	def do_svn(self,widget,event):
		cmd = 'cd ~/saliency;svn up;cd -'
		self.cmd_all_checked_tabs(cmd)
	def on_edit_copy(self,widget):
		id = self.get_current_ID()
		v[id].copy_clipboard()

	def on_edit_paste(self,widget):
		#Method 1
		id = self.get_current_ID()
		clipboard = gtk.clipboard_get()
		text = clipboard.wait_for_text()
		v[id].feed_child(text)
		#Method 2
		#v[id].paste_clipboard()

	def on_close_current_tab(self,widget)	:
		nth = self.tabs.get_current_page()
		child = self.tabs.get_nth_page(nth)
		self.tabs.close_tab(widget,child)

	def on_host_select(self,widget)	:
		for i in range(8):
			buf = "bx%d" % (i+1)
			self.menu_hosts_c[buf].set_active(widget.active)
	def on_host_open_all(self,widget)	:
		for i in range(8):
			buf = "bx%d" % (i+1)
			if(self.menu_hosts_c[buf].active):
				self.tabs.insert_tab(buf)

	def on_host_open(self,widget,host)	:
		print "Open New "+host
		self.tabs.insert_tab(host)
	def on_host_open_load(self,widget,load_list):
		self.tabs.close_all_tabs()
		#not in order, need fix later
		for name,h in sorted(load_list.items()):
			print "host["+h+"]  " + name
			self.tabs.insert_tab_with_name(h,name)

	def on_lock_tab(self,widget):

		num = self.tabs.get_n_pages()
		for i in range(int(num)):
			frame = self.tabs.get_nth_page(i)
			label = self.tabs.get_tab_label(frame)
			self.tabs.set_tab_reorderable(frame,not widget.active)

	def on_select_tab(self,widget):
		for id,cb in self.tabs.check_button.iteritems():
			cb.set_active(widget.active)
	def responseToDialog(self,entry,dialog,response):
		print response
		dialog.response(response)
	def on_rename_tab(self,widget):		
		dialog = gtk.MessageDialog(
				None,
				gtk.DIALOG_MODAL | gtk.DIALOG_DESTROY_WITH_PARENT,
				gtk.MESSAGE_QUESTION,
				gtk.BUTTONS_OK,
				None)
		dialog.set_markup("Please Enter tab name:")
		entry = gtk.Entry()
		id = self.get_current_ID()
		entry.set_text(self.tabs.label[id].get_text())
		entry.connect("activate",self.responseToDialog,dialog,gtk.RESPONSE_OK)
		dialog.vbox.pack_end(entry)
		dialog.show_all()
		dialog.run()
		text = entry.get_text()
		dialog.destroy()
		if(text != ""):
			id = self.get_current_ID()
			self.tabs.label[id].set_text(text)
	def on_pre_tab(self,widget):		
		self.tabs.prev_page()
	def on_next_tab(self,widget):
		self.tabs.next_page()
	def on_left_tab(self,widget):
		nth = self.tabs.get_current_page()
		child = self.tabs.get_nth_page(nth)
		if(nth != 0):
			self.tabs.reorder_child(child,nth-1)
	def on_right_tab(self,widget):
		nth = self.tabs.get_current_page()
		child = self.tabs.get_nth_page(nth)
		self.tabs.reorder_child(child,nth+1)
	def on_switch_page(self,nb,page,page_num):
		#hide original tab close button
		id = self.get_current_ID()
		self.tabs.close_button[id].hide()
		self.tabs.label2[id].show()
		self.tabs.close_button[id].set_sensitive(False)
		#show new focus tab close button
		current_frame = self.tabs.get_nth_page(page_num)
		id = current_frame.get_label()
		self.tabs.close_button[id].show()
		self.tabs.label2[id].hide()
		self.tabs.close_button[id].set_sensitive(True)
	def key_press_event(self,widget,event):
		from gtk.gdk import CONTROL_MASK
		from gtk.gdk import keyval_name
		#if event.state & CONTROL_MASK:
			#if keyval_name(event.keyval) == "Page_Up":
			#elif keyval_name(event.keyval) == "Page_Down":
			#else:
		print "Your press " + keyval_name(event.keyval) + " " + str(event.keyval)
if __name__ == '__main__':
	beoTerminal()
 	gtk.main()
