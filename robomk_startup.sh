#!/bin/bash
# Script para abrir Guake con 5 pestañas, cada una en su directorio, con comandos opcionales y nombres visibles

# Inicia Guake si no está abierto
guake &

# Espera unos segundos a que Guake se inicialice
sleep 3

# --- CONFIGURA AQUÍ TUS DIRECTORIOS ---
DIR_WebotsBridge="$HOME/robocomp/components/webots-bridge"
DIR_Joystic="$HOME/robocomp/components/robocomp-robolab/components/hardware/external_control/joysticpublish"
DIR_Lidar3D="$HOME/robocomp/components/robocomp-robolab/components/hardware/laser/lidar3D"
DIR_Chocachoca="$HOME/robomk/actividad1"

# --- CONFIGURA AQUÍ LOS COMANDOS QUE QUIERAS EJECUTAR ---
DIR_WebotsBridge="bin/Weebots2Robocomp etc/config"
DIR_Joystic="bin/JoysticPublish etc/config_shadow"
DIR_Lidar3D="bin/Lidar3D_helios_webots etc/config"        
DIR_Chocachoca="bin/chocachoca etc/config"       

# --- NOMBRES DE LAS PESTAÑAS ---
NAME_WebotsBridge="WebotsBridge"
NAME_Joystic="Joystic"
NAME_Lidar3D="Lidar3D"
NAME_Chocachoca="Chocachoca"

# --- CREA LAS 5 PESTAÑAS ---
# Formato: guake -n <TAB_INDEX> -e "<comando>" -r "<nombre de la pestaña>"
[ -n "$CMD_WebotsBridge" ] && guake -n 0 -e "cd $DIR_WebotsBridge; $CMD_WebotsBridge" -r "$NAME_WebotsBridge" || guake -n 0 -e "cd $DIR_WebotsBridge" -r "$NAME_WebotsBridge"
[ -n "$CMD_Joystic" ] && guake -n 1 -e "cd $DIR_Joystic; $CMD_Joystic" -r "$NAME_Joystic" || guake -n 1 -e "cd $DIR_Joystic" -r "$NAME_Joystic"
[ -n "$CMD_Lidar3D" ] && guake -n 2 -e "cd $DIR_Lidar3D; $CMD_Lidar3D" -r "$NAME_Lidar3D" || guake -n 2 -e "cd $DIR_Lidar3D" -r "$NAME_Lidar3D"
[ -n "$CMD_Chocachoca" ] && guake -n 3 -e "cd $DIR_Chocachoca; $CMD_Chocachoca" -r "$NAME_Chocachoca" || guake -n 3 -e "cd $DIR_Chocachoca" -r "$NAME_Chocachoca"

# Muestra Guake en pantalla
guake --show

