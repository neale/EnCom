# EnCom
Modular RS232 Monitor 

## Installation 

clone the repository to your project directory 
```sh
git clone http://github.com/neale/EnCom.git

```
# Compile Project

navigate to the source and compile 

```sh 
cd EnCom/
gcc -Wall serialMonitor.c -o monitor
```

#Usage

Execute `./monitor ...` with options:
* -b Baud rate 
* -d data bits (1 or 2)
* -p parity (p or n)
If no options are given default options are used `8n1 38400`

## Support

Please [open an issue](https://github.com/neale/EnCom/issues/new) for questions and concerns.

## Contributing 

Fork the project, commit your changes, and open a [open a pull request](https://github.com/neale/EnCom/compare/).
