#!/bin/bash

# ========== CONFIGURACIÓN ==========
BAG_IN_DIR="records"
BAG_FILE="$BAG_IN_DIR/$1"
BAG_START="$2"
BAG_END="$3"

MOV_IN_DIR="Videos_pruebas"
MOV_NAME_FILE="$4"
MOV_FILE="$MOV_IN_DIR/$4"
MOV_START="$5"
MOV_END="$6"

ROOT_OUT_DIR="extracted_frames"
BAG_OUT_DIR="$ROOT_OUT_DIR/frames_from_bag"
MOV_OUT_DIR="$ROOT_OUT_DIR/frames_from_mov"

IMAGE_TOPIC="/inference_result"  # Asegúrate que este es tu topic correcto

FINAL_FRAMERATE=15

# ========== FUNCIONES ==========


extract_from_bag() {
  echo "▶️ Extrayendo frames del bag: $BAG_FILE desde $BAG_START hasta $BAG_END segundos..."

  mkdir -p "$BAG_OUT_DIR"
  BAG_DURATION=$(echo "$BAG_END - $BAG_START" | bc)

  # Lanzar image_saver en background con argumentos correctos
  (
    sleep 1  # Espera breve para asegurar sincronía
    ros2 run image_view image_saver --ros-args --remap image:="$IMAGE_TOPIC" --param filename_format:="$BAG_OUT_DIR/frame_%05d.png"
  ) &
  VIEW_PID=$!

  # Lanzar el bag pausado
  ros2 bag play "$BAG_FILE" --start-offset "$BAG_START" --start-paused &
  BAG_PID=$!

  echo "⏳ Esperando a que el rosbag esté listo..."
  sleep 20
  
  # Reanudar
  echo "▶️ Reanudando reproducción..."
  ros2 service call /rosbag2_player/resume rosbag2_interfaces/srv/Resume "{}"
	

  # Esperar duración deseada
  sleep "$BAG_DURATION"
  
  # Pausar
  echo "▶️ Pausando reproducción..."
  if ros2 service list | grep '/rosbag2_player/pause'; then
    echo "⏸️ Pausando rosbag..."
    ros2 service call /rosbag2_player/pause rosbag2_interfaces/srv/Pause "{}"
  else
    echo "⚠️ El servicio de pause no está disponible (rosbag ya terminó)."
  fi

  kill $BAG_PID
  kill $VIEW_PID
  pkill -f image_saver
  echo "✅ Extracción desde bag finalizada."
}

create_video_from_bag_frames() {
  echo "▶️ Creando vídeo desde frames del bag..."

  # Contar frames extraídos
  NUM_FRAMES=$(ls "$BAG_OUT_DIR"/frame_*.png 2>/dev/null | wc -l)
  echo "ℹ️ Se han extraído $NUM_FRAMES frames del bag."

  if [ "$NUM_FRAMES" -eq 0 ]; then
    echo "⚠️ No hay frames para crear video."
    return
  fi

  # Calcular framerate original aproximado para esos frames y duración
  BAG_DURATION=$(echo "$BAG_END - $BAG_START" | bc)
  ORIG_FPS=$(echo "scale=2; $NUM_FRAMES / $BAG_DURATION" | bc)

  echo "ℹ️ Framerate original aproximado: $ORIG_FPS fps para duración $BAG_DURATION segundos."

  # Crear video de 30 fps y duración exacta BAG_DURATION,
  # usando -r para salida y -framerate para entrada
  ffmpeg -y -framerate "$ORIG_FPS" -i "$BAG_OUT_DIR/frame_%05d.png" -r $FINAL_FRAMERATE -t "$BAG_DURATION" -pix_fmt yuv420p "$BAG_OUT_DIR/bag_video_30fps.mp4"

  echo "✅ Vídeo creado en $BAG_OUT_DIR/bag_video_30fps.mp4"
}

extract_from_mov() {
  echo "🎬 Extrayendo frames del video: $MOV_FILE desde $MOV_START hasta $MOV_END..."

  mkdir -p "$MOV_OUT_DIR"
  ffmpeg -ss "$MOV_START" -to "$MOV_END" -i "$MOV_FILE" -vf fps=$FINAL_FRAMERATE "$MOV_OUT_DIR/frame_%05d.png"

  echo "✅ Extracción desde .mov finalizada."
}

create_video_from_mov_frames() {
  echo "▶️ Creando vídeo desde frames del mov..."

  NUM_FRAMES=$(ls "$MOV_OUT_DIR"/frame_*.png 2>/dev/null | wc -l)
  echo "ℹ️ Se han extraído $NUM_FRAMES frames del mov."

  if [ "$NUM_FRAMES" -eq 0 ]; then
    echo "⚠️ No hay frames para crear video."
    return
  fi

  MOV_DURATION=$(echo "$MOV_END - $MOV_START" | bc)

  ffmpeg -y -framerate $FINAL_FRAMERATE -i "$MOV_OUT_DIR/frame_%05d.png" -r $FINAL_FRAMERATE -t "$MOV_DURATION" -pix_fmt yuv420p "$MOV_OUT_DIR/mov_video_30fps.mp4"

  echo "✅ Vídeo creado en $MOV_OUT_DIR/mov_video_30fps.mp4"
}

combine_videos_side_by_side() {
  echo "▶️ Combinando videos..."

  echo "$MOV_NAME_FILE"
  local mov_name="$(basename "$MOV_NAME_FILE")"
  echo "$mov_name"
  local base_name="${mov_name%.MOV}"                # Quitar .mov
  echo "$base_name"
  local num="$(echo "$base_name" | grep -oE '[0-9]+$')"  # Extraer número final
  echo "$num"
  local name_no_num="${base_name%"$num"}"            # Quitar número del final
  echo "$name_no_num"
  local OUTPUT_VIDEO="$ROOT_OUT_DIR/${name_no_num}multiples_vistas_$num.mp4"
  echo "$OUTPUT_VIDEO"

  ffmpeg -y \
    -i "$MOV_OUT_DIR/mov_video_30fps.mp4" \
    -i "$BAG_OUT_DIR/bag_video_30fps.mp4" \
    -filter_complex "\
      [1:v]scale=iw*0.4:-1[small]; \
      [0:v][small]overlay=W-w-30:30" \
    -c:v libx264 -crf 18 -preset veryfast "$OUTPUT_VIDEO"


  echo "✅ Vídeo final guardado en $OUTPUT_VIDEO"
}



# ========== EJECUCIÓN ==========

if [ -n "$BAG_FILE" ] && [ -d "$BAG_FILE" ]; then
  extract_from_bag
  create_video_from_bag_frames
else
  echo "⚠️ Bag file no especificado o no encontrado: $BAG_FILE"
fi

if [ -n "$MOV_FILE" ] && [ -f "$MOV_FILE" ]; then
  extract_from_mov
  create_video_from_mov_frames
else
  echo "⚠️ MOV file no especificado o no encontrado: $MOV_FILE"
fi

# Solo combinamos si ambos vídeos existen:
if [ -f "$BAG_OUT_DIR/bag_video_30fps.mp4" ] && [ -f "$MOV_OUT_DIR/mov_video_30fps.mp4" ]; then
  combine_videos_side_by_side
else
  echo "⚠️ No se pueden combinar videos: falta alguno."
fi

rm -rf "$BAG_OUT_DIR"
rm -rf "$MOV_OUT_DIR"

