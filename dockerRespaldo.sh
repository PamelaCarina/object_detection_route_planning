#!/bin/bash

# -------------------------
# CONFIGURACIÓN INICIAL
# -------------------------

# Nombres de contenedor e imagen
NEW_CONTAINER_NAME="yolov5_contenedor"
NEW_IMAGE_NAME="yolov5_imagen"


# Carpeta para respaldos
BACKUP_DIR="./respaldos"

# -------------------------
# FUNCIÓN: Verificar si pv está instalado
# -------------------------
check_pv() {
  if ! command -v pv &>/dev/null; then
    echo "'pv' no está instalado. ¿Quieres instalarlo? (s/n)"
    read -r respuesta
    if [[ "$respuesta" == "s" || "$respuesta" == "S" ]]; then
      echo "Instalando pv..."
      sudo apt update && sudo apt install -y pv
      if [[ $? -ne 0 ]]; then
        echo "Error al instalar 'pv'. Por favor instalalo manualmente."
        exit 1
      fi
    else
      echo "Continuando sin barra de progreso."
    fi
  fi
}

# -------------------------
# RESPALDO (COMMIT + SAVE CON PV)
# -------------------------

mkdir -p "$BACKUP_DIR"

CONTAINER_ID=$(docker ps -q -f name="$NEW_CONTAINER_NAME")
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
BACKUP_IMAGE_TAG="${NEW_IMAGE_NAME}:${TIMESTAMP}"
BACKUP_FILE="${BACKUP_DIR}/${NEW_IMAGE_NAME}_${TIMESTAMP}.tar"

echo ""
echo "Iniciando respaldo..."
echo "Contenedor: $NEW_CONTAINER_NAME (ID: $CONTAINER_ID)"
echo "Nueva imagen: $BACKUP_IMAGE_TAG"
echo "Archivo de respaldo: $BACKUP_FILE"

# Crear imagen desde el contenedor
docker commit "$CONTAINER_ID" "$BACKUP_IMAGE_TAG"
if [[ $? -ne 0 ]]; then
  echo "Error al hacer commit."
  exit 1
fi
echo "Commit exitoso."

# Verificar e instalar pv si se desea
check_pv

echo "Guardando imagen con barra de progreso..."

# Usar pv si está disponible
if command -v pv &>/dev/null; then
  docker save "$BACKUP_IMAGE_TAG" | pv > "$BACKUP_FILE"
else
  docker save -o "$BACKUP_FILE" "$BACKUP_IMAGE_TAG"
fi

if [[ $? -ne 0 ]]; then
  echo "Error al guardar la imagen."
  exit 1
fi

echo "Imagen guardada en: $BACKUP_FILE"
echo "Respaldo completado con exito."