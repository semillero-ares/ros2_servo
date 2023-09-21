## Comunicación serial en WSL

En WSL necesitaremos instalar [esto](https://learn.microsoft.com/en-us/windows/wsl/connect-usb). O tendremos que generar una comunicación serial falsa de la siguiente forma

```bash
socat -d -d pty,rawer,echo=0 pty,rawer,echo=0
```

En mi caso aparecieron los puertos seriales `/dev/pts/3` y `/dev/pts/4`... los cuales pueden ser usados en el archivo de configuración de ros2_control.


## Conexión por TCP al serial de Windows


Para conectarnos por TCP a un serial de Windows desde nuestro Ubuntu WSL, vamos a usar dos programas, uno en windows y otro en linux, 

En windows podremos usar:

- [TCPCOM32](https://sourceforge.net/projects/tcpcom32/), se instala y se configura como server. 
- [com0com](https://com0com.sourceforge.net/)

más referencias [aquí](https://superuser.com/questions/54723/any-free-application-to-redirect-serial-communication-to-tcp-ip)

En linux podremos usar multiples opciones:

- [ser2net](https://manpages.ubuntu.com/manpages/jammy/en/man8/ser2net.8.html)

    ```bash
    sudo apt-get install ser2net
    ```

- [socat](https://www.digi.com/support/knowledge-base/serial-to-ethernet-or-wifi-bridge-with-linux-socat) ([ref-2](http://www.dest-unreach.org/socat/doc/socat-ttyovertcp.txt))

[REVISAR](https://stackoverflow.com/questions/484740/converting-serial-port-data-to-tcp-ip-in-a-linux-environment)

Como vamos a estar trabajando con Arduino, la configuración serial de este es: 

**[SERIAL_8N1](https://www.arduino.cc/reference/en/language/functions/communication/serial/begin/) :**
Comunicación serial de 8 Bits, Sin paridad (N), con un bit de parada.