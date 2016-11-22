# Firmware Localización en Cuevas
[![Localizacion en cuevas](https://cldup.com/QvZZtmnKTN.png)](http://www.localizacionencuevas.com)

Este repositorio contiene el código fuente del firwmare necesario para ejecutar la aplicación sobre el hardware de localización en cuevas que conforma la red de sensores.

La plataforma de desarrollo que se ha utilizado es [Atmel Studio 7](http://www.atmel.com/Microsite/atmel-studio/), dentro del directorio apps hay 3 soluciones que contienen los siguientes componentes:

  - Bootloader, versión modificado para permitir actualizaciones aéreas.
  - Servidor OTA, servidor responsable de realizar actualizaciones aéreas.
  - Aplicación, contiene toda la lógica de la aplicación y se ha utilizado como base la plataforma [Atmel Lightweight Mesh](http://www.atmel.com/tools/lightweight_mesh.aspx).

Este proyecto ha sido cofinanciado por la **Junta de Comunidades de Castilla la Mancha** bajo el número de expediente 1615ITA136.

[![Logo JCCM](https://cldup.com/y-bVZMxRzA.JPG)](http://www.jccm.es)
