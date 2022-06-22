TUTORIAL PARA Não fazer nada quando fechar a tampa
Por exemplo na SARA você pode conectar o notebook no monitor e fechar a tampa, todos os processos vão ficar rodando normalmente.
Primeiro eu desabilitei o lock screen e o screen saver, mas talvez nao precise, vou manter na ordem que fiz:
Alguem poderia testar só a partir da linha 65 e editar esse README

****************************************
***-Ubuntu – Disabling Lock Screen 18.04

I've just installed Ubuntu 18.04 and I want to disable the lock screen. I don't want to see it at all, neither on power-on nor after suspension.

Disable Lock Screen

You can disable the lock screen permanently when waking from suspend.

First use this command to discover current settings:

$ gsettings get org.gnome.desktop.lockdown disable-lock-screen
false

Now set it to true using this command:

gsettings set org.gnome.desktop.lockdown disable-lock-screen 'true'

If you are unhappy with the new setting you can reverse it using:

gsettings set org.gnome.desktop.lockdown disable-lock-screen 'false'

****************************************
***-Disable Screen Saver Locking

There was some confusion where people think disabling the Lock screen also disables the screen saver which is invoked after a certain period of inactivity. The screen saver requires input to get your desktop back. Some people may want the screen saver to come on but not have it locked when waking up the screen.

To check screen saver lock status use:

$ gsettings get org.gnome.desktop.screensaver lock-enabled
true

If true you can turn off screen saver locking with:

gsettings set org.gnome.desktop.screensaver lock-enabled false

To reverse the setting back use:

gsettings set org.gnome.desktop.screensaver lock-enabled true

In Gnome screen locking guide it says:

    6.2. Screen Locking

    By default, GNOME Power Manager supports a simple locking scheme. This means that the screen will lock if set to Lock screen in gnome-screensaver when the lid is closed, or the system performs a suspend or hibernate action.

    There is a complex locking scheme available for power users that allows locking policy to change for the lid, suspend and hibernate actions. To enable this complex mode, you will have to disable the GConf key:

        /apps/gnome-power-manager/lock/use_screensaver_settings

    Then the policy keys can be set to force a gnome-screensaver lock and unlock when the action is performed:

        /apps/gnome-power-manager/lock/blank_screen
        /apps/gnome-power-manager/lock/suspend
        /apps/gnome-power-manager/lock/hibernate


Fonte: https://itectec.com/ubuntu/ubuntu-disabling-lock-screen-18-04/

****************************************
*** -Nao Fazer nada quando fechar a tampa do notebook

For 13.10 - 21.04:
To make Ubuntu do nothing when laptop lid is closed:

Open the /etc/systemd/logind.conf file in a text editor as root, for example,

 sudo -H gedit /etc/systemd/logind.conf
If HandleLidSwitch is not set to ignore then change it:

 HandleLidSwitch=ignore
Make sure it's not commented out (it is commented out if it is preceded by the symbol #) or add it if it is missing,

Restart the systemd daemon (be aware that this will log you off) with this command:

sudo systemctl restart systemd-logind
or, from 15.04 onwards:

sudo service systemd-logind restart

Nao testei os passos abaixo, mas se não funcionar, pode ser uma opção:
For GNOME Users:

If you are using GNOME (the default in 18.04+), then you can do this easily without changing system settings by using the "Gnome Tweak Tool". It can be installed from the Ubuntu Software Store (It is called GNOME Tweaks). Or if you prefer the console: sudo apt-get install gnome-tweak-tool

Run it after installing, then under Power, Turn off the setting to do nothing when lid is closed. I tested this on Ubuntu 18.04 and it works.

Fonte: https://askubuntu.com/questions/15520/how-can-i-tell-ubuntu-to-do-nothing-when-i-close-my-laptop-lid
