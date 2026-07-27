#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# Prepara docker/glim/config/ para el ejemplo de SLAM 3D.
#
# Por que no hay una carpeta de config ya hecha en el repo:
# GLIM usa 14 ficheros .json y sus claves cambian entre versiones. En vez de
# congelar una copia que se quedaria vieja, este script:
#
#   1. saca la configuracion ORIGINAL de la imagen Docker que tengais
#   2. le mezcla encima nuestros cambios (overrides.json)
#
# Asi la config siempre corresponde a la version de GLIM que vais a ejecutar.
#
#   ./preparar_config.sh                      # imagen CPU (por defecto)
#   ./preparar_config.sh koide3/glim_ros2:humble_cuda12.2
# ---------------------------------------------------------------------------
set -euo pipefail

IMAGEN="${1:-koide3/glim_ros2:humble}"
AQUI="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DESTINO="$AQUI/config"
ORIGEN_EN_IMAGEN="/root/ros2_ws/install/glim/share/glim/config"

echo "Imagen : $IMAGEN"
echo "Destino: $DESTINO"

if ! docker image inspect "$IMAGEN" > /dev/null 2>&1; then
    echo "La imagen no esta descargada. Bajandola (son ~2.4 GB)..."
    docker pull "$IMAGEN"
fi

rm -rf "$DESTINO"
mkdir -p "$DESTINO"

# docker cp necesita un contenedor, no una imagen: creamos uno sin arrancarlo
CID=$(docker create "$IMAGEN")
trap 'docker rm -f "$CID" > /dev/null 2>&1 || true' EXIT
docker cp "$CID:$ORIGEN_EN_IMAGEN/." "$DESTINO/"

echo "Configuracion original extraida:"
ls "$DESTINO" | sed 's/^/  /'

python3 - "$DESTINO" "$AQUI/overrides.json" <<'PYTHON'
"""Mezcla overrides.json sobre los .json de GLIM.

Los ficheros de GLIM llevan comentarios (// y /* */), que json no acepta,
asi que hay que limpiarlos antes de parsear. El resultado se escribe como
JSON normal, que GLIM tambien lee sin problema.
"""
import json
import sys
from pathlib import Path

destino, overrides_path = Path(sys.argv[1]), Path(sys.argv[2])


def quitar_comentarios(texto):
    fuera = []
    i, n = 0, len(texto)
    en_cadena = False
    while i < n:
        c = texto[i]
        if en_cadena:
            fuera.append(c)
            if c == "\\" and i + 1 < n:      # escape dentro de la cadena
                fuera.append(texto[i + 1])
                i += 2
                continue
            if c == '"':
                en_cadena = False
            i += 1
        elif c == '"':
            en_cadena = True
            fuera.append(c)
            i += 1
        elif texto.startswith("//", i):
            i = texto.find("\n", i)
            if i < 0:
                break
        elif texto.startswith("/*", i):
            fin = texto.find("*/", i + 2)
            i = n if fin < 0 else fin + 2
        else:
            fuera.append(c)
            i += 1
    return "".join(fuera)


def mezclar(base, nuevo):
    for k, v in nuevo.items():
        if k.startswith("_"):            # claves de documentacion nuestras
            continue
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            mezclar(base[k], v)
        else:
            base[k] = v
    return base


overrides = json.loads(quitar_comentarios(overrides_path.read_text()))

for fichero, cambios in overrides.items():
    if fichero.startswith("_"):
        continue
    ruta = destino / fichero
    if not ruta.exists():
        print(f"  AVISO: {fichero} no existe en esta version de GLIM, saltando")
        continue
    original = json.loads(quitar_comentarios(ruta.read_text()))
    ruta.write_text(json.dumps(mezclar(original, cambios), indent=2) + "\n")

    def hojas(d, pre=""):
        for k, v in d.items():
            if k.startswith("_"):
                continue
            if isinstance(v, dict):
                yield from hojas(v, f"{pre}{k}.")
            else:
                yield f"{pre}{k} = {v}"

    print(f"  {fichero}:")
    for linea in hojas(cambios):
        print(f"      {linea}")
PYTHON

echo
echo "Listo. Ahora:"
echo "    docker compose -f ../docker-compose.yml up glim"
