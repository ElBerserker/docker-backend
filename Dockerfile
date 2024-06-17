# Utilizar una imagen base de Python
FROM python:3.8-slim

# Establecer el directorio de trabajo dentro del contenedor
WORKDIR /app

# Copiar el archivo requirements.txt al contenedor
COPY requirements.txt requirements.txt

# Instalar las dependencias de Python
RUN pip install --no-cache-dir -r requirements.txt

# Copiar todo el contenido del directorio actual al contenedor
COPY . .

EXPOSE 5000
# Exponer el puerto en el que se ejecutará la aplicación Flask

# Comando para ejecutar la aplicación
CMD ["python", "app.py"]

