---------------------------------------
Pasos para ejecutar el prototipo
---------------------------------------
1. Levantar los contenedores docker
2. Importar flow en NodeRed
3. Importar Dashboard en Grafana
4. Modificar dirección IP en el código main.cpp y ejecución


**********************************************
1. Manejo de los contenedores de docker
**********************************************
1.1. Modificar la linea 36 del archivo docker-compose.yml por la ruta del equipo donde se tenga el proyecto
1.2. docker-compose up   //Levantar los contenedores
	docker-compose down // Bajar los contenedores

**********************************************
2. Importar flow en NodeRed
**********************************************
2.1. Abrir navegador e ir a la página localhost:1880
2.2. Importar el archivo flows proto.json
2.3. Modificar IP en el contenedor mqtt subscriber del flow.
2.4. Ejecutar en el directorio del proyecto: docker cp src/nodeRed.proto mynodered:/data


**********************************************
3. Importar Dashboard en Grafana
**********************************************
3.1. Abrir navegador e ir a la página localhost:3000
3.2. Modificar el origen de datos de grafana para coger los datos de Influxdb
	- En esta pagina incluimos como url: http://<dirección_IP>:8086
	- Nombre base de datos: g5
3.3. Importar el archivo Dashboard G5-1612006908107.json

***********************************************
4. Modificar dirección IP en el código main.cpp
************************************************
4.1. Modificar linea 22 añadiendo la IP del equipo que estamos usando
4.2. Modificar linea 19 cambiando el nombre de la red que se tenga
4.3. Modificar linea 20 con la contraseña de la red 
4.4. Ejecutar programa en Wemos con CTRL+ALT+U 

--------------------------------------
Ejecución de los casos de uso
---------------------------------------
A.	Regado automático de los cultivos
Este caso de uso se ejecuta automáticamente ejecutando el programa en la Wemos.


B.	Consulta de estadísticas mediante Dashboard Grafana. 
Abrir navegador y navegar a la página "localhost:3000" 

Seleccionar dashboard importado donde podremos ver estadísticas en función del tiempo.

C.	 Consulta de valores actuales mediante Dashboards NodeRed.
Abrir navegador en la página localhost:1880/ui

D.	 Regado de los cultivos a petición del agricultor.
Este caso de uso se modela pulsando el botón de la placa wemos situado en el pin D2


