# ar       - compacta os objetos criados em um arquivo .a
# ranlib   - indexa a biblioteca para agilizar o processo de linker com bibliotecas estáticas
# size     - apresenta a quantidade de memória ocupada pela biblioteca
# -g       - adds debugging information to the executable file
# -Ox      - defines compiler optimization level
# -Wall    - disable almost all warning flags
# -pipe	   - use pipes rather than temporary files between compilation stages
# -Winline - informs if the functions with inline directives has not been inlined and why not.


TARGET=libmmodbus.so

DEBUG	= -O3
CC	= gcc
INCLUDE	= -I.
CFLAGS	= $(DEBUG) -Wall $(INCLUDE) -Winline -pipe -fPIC -shared -fpic

LIBS    = 

# Should not alter anything below this line
###############################################################################

SRC	=	libmmodbus.c

OBJ	=	libmmodbus.o

all:		$(TARGET)

$(TARGET):	$(OBJ)
		$(CC) -shared -fpic -o $(TARGET) $(OBJ)

.c.o:
	@echo [CC] $<
	@$(CC) -I/usr/lib/jvm/java-8-openjdk-amd64/include -I/usr/lib/jvm/java-8-openjdk-amd64/include/linux -c $(CFLAGS) $< -o $@

clean:
	rm -f $(OBJ) $(TARGET)

