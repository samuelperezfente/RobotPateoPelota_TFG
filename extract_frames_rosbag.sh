#!/bin/bash

# ========== CONFIGURACIÓN ==========
BAG_IN_DIR="records"
BAG_NAME="$1"
BAG_FILE="$BAG_IN_DIR/$1"
BAG_START="$2"
BAG_END="$3"

OUT_DIR_BASE="extracted_frames"

# Lista de tópicos
TOPIC1="/inference_result"
TOPIC2="/usb_cam_0/image_raw"
TOPIC3="/usb_cam_1/image_raw"

# Directorios de salida
OUT1="$OUT_DIR_BASE/robot"
OUT2="$OUT_DIR_BASE/usb_cam0"
OUT3="$OUT_DIR_BASE/usb_cam1"

FASES_FILE="$OUT_DIR_BASE/fases.txt"
FILTER_FILE="$OUT_DIR_BASE/filtro_ffmpeg.txt"

MAIN_SCALE="1280:720"
SMALL_SCALE="320:180"
SMALL_POSITION1="W-w-20:20"
SMALL_POSITION2="20:20"

PHASE_POSITION_X="(w-text_w)/2"
PHASE_POSITION_Y="20"

SUBPHASE_POSITION_X="(w-text_w)/2"
SUBPHASE_POSITION_Y="60"


VAR_POSITION_X="20"
VAR_Y_OFFSET=220
VAR_LINE_HEIGHT=40



# ROOT_OUT_DIR="extracted_frames"
# BAG_OUT_DIR="$ROOT_OUT_DIR/frames_from_bag"
# MOV_OUT_DIR="$ROOT_OUT_DIR/frames_from_mov"

# IMAGE_TOPIC="/inference_result"  # Asegúrate que este es tu topic correcto

FINAL_FRAMERATE=30

# ========== FUNCIONES ==========
extract_from_bag() {
  echo "▶️ Extrayendo frames del bag: $BAG_FILE desde $BAG_START hasta $BAG_END segundos..."

  mkdir -p "$OUT1" "$OUT2" "$OUT3"
  BAG_DURATION=$(echo "$BAG_END - $BAG_START" | bc)

  # Lanzar image_saver para cada tópico en su grupo de proceso
  (sleep 1; ros2 run image_view image_saver --ros-args --remap image:="$TOPIC1" --param filename_format:="$OUT1/frame_%05d.png") &
  PID1=$!
  # echo $PID1
  # PGID1=$(ps -o pgid= $PID1 | tr -d ' ')

  (sleep 1; ros2 run image_view image_saver --ros-args --remap image:="$TOPIC2" --param filename_format:="$OUT2/frame_%05d.png") &
  PID2=$!
  # echo $PID2
  # PGID2=$(ps -o pgid= $PID2 | tr -d ' ')

  (sleep 1; ros2 run image_view image_saver --ros-args --remap image:="$TOPIC3" --param filename_format:="$OUT3/frame_%05d.png") &
  PID3=$!
  # echo $PID3
  # PGID3=$(ps -o pgid= $PID3 | tr -d ' ')

  # Lanzar rosbag play pausado en su propio grupo
  ros2 bag play "$BAG_FILE" --start-offset "$BAG_START" --start-paused &
  BAG_PID=$!
  # echo $BAG_PID
  # BAG_PGID=$(ps -o pgid= $BAG_PID | tr -d ' ')

  echo "⏳ Esperando a que el rosbag esté listo..."
  sleep 20
  
  # Reanudar rosbag
  echo "▶️ Reanudando reproducción..."
  ros2 service call /rosbag2_player/resume rosbag2_interfaces/srv/Resume "{}"
  START_TIME=$(date +%s.%N)

  # Capturar mensajes rosout en segundo plano (misma idea)
  (
    ros2 topic echo /rosout -f | stdbuf -oL awk -v start_time="$START_TIME" -v fases_file="$FASES_FILE" '
    /Fase/ {
      # Llamar a date por cada línea es costoso, mejor pasarlo desde shell:
      # Aquí awk solo imprime y guarda, pero el tiempo debe venir con la línea.
      print $0
      # No hace nada aquí, se usará en el siguiente paso
    }
    ' | while IFS= read -r line; do
      NOW=$(date +%s.%N)
      REL_TIME=$(echo "$NOW - $START_TIME" | bc)
      echo "$line"
      echo "$REL_TIME;$line" >> "$FASES_FILE"
    done
  ) &

  PID_FASES=$!
  # echo $PID_FASES
  # PGID_FASES=$(ps -o pgid= $PID_FASES | tr -d ' ')

  # Esperar duración deseada
  sleep "$BAG_DURATION"
  
  # Pausar rosbag
  echo "▶️ Pausando reproducción..."
  if ros2 service list | grep -q '/rosbag2_player/pause'; then
    echo "⏸️ Pausando rosbag..."
    ros2 service call /rosbag2_player/pause rosbag2_interfaces/srv/Pause "{}"
  else
    echo "⚠️ El servicio de pause no está disponible (rosbag ya terminó)."
  fi

  END_TIME=$(date +%s.%N)

  REAL_DUR=$(echo "$END_TIME - $START_TIME" | bc)
  echo "⏱️ Duración real de reproducción: $REAL_DUR segundos"

  BAG_REAL_DURATION="$REAL_DUR"

  echo "🛑 Matando todos los grupos de procesos..."

  # Matar grupos de procesos (con signo menos)
  kill $PID1 $PID2 $PID3 $BAG_PID $PID_FASES

  # echo "PIDS: $PID1 $PID2 $PID3 $BAG_PID $PID_FASES"

  # Por si queda algo colgado, matar image_saver suelto

  echo "✅ Extracción desde bag finalizada."
}

create_video_from_bag_frames() {
  local frames_dir="$1" 

  echo "▶️ Creando vídeo desde frames del bag..."

  # Contar frames extraídos
  NUM_FRAMES=$(ls "$frames_dir"/frame_*.png 2>/dev/null | wc -l)
  echo "ℹ️ Se han extraído $NUM_FRAMES frames del bag."

  if [ "$NUM_FRAMES" -eq 0 ]; then
    echo "⚠️ No hay frames para crear video."
    return
  fi

  # Calcular framerate original aproximado para esos frames y duración
  BAG_DURATION=$(echo "$BAG_END - $BAG_START" | bc)
  ORIG_FPS=$(echo "scale=2; $NUM_FRAMES / $BAG_REAL_DURATION" | bc)

  echo "ℹ️ Framerate original aproximado: $ORIG_FPS fps para duración $BAG_REAL_DURATION segundos."

  # Crear video de 30 fps y duración exacta BAG_DURATION,
  # usando -r para salida y -framerate para entrada
  ffmpeg -y -framerate "$ORIG_FPS" -i "$frames_dir/frame_%05d.png" -r $FINAL_FRAMERATE -t "$BAG_REAL_DURATION" -pix_fmt yuv420p "$frames_dir/bag_video_30fps.mp4"

  echo "✅ Vídeo creado en $frames_dir/bag_video_30fps.mp4"
}

combine_videos_side_by_side() {
  echo "▶️ Combinando videos..."
  local OUTPUT_VIDEO="$OUT_DIR_BASE/$BAG_NAME.mp4"
  

  echo "ℹ️ Generando filtro en $FILTER_FILE"
  : > "$FILTER_FILE"  # limpiar archivo

  # Filtro base inicial con escalado y overlays
  {
    echo "[0:v]scale=$MAIN_SCALE[main];"
    echo "[1:v]scale=$SMALL_SCALE[small1];"
    echo "[2:v]scale=$SMALL_SCALE[small2];"
    echo "[main][small1]overlay=$SMALL_POSITION1[tmp1];"
    echo "[tmp1][small2]overlay=$SMALL_POSITION2"
  } >> "$FILTER_FILE"

  if [ -f "$FASES_FILE" ]; then
    echo "ℹ️ Añadiendo texto (todas las líneas sin filtrar cambios) desde $FASES_FILE"
    mapfile -t LINES < "$FASES_FILE"

    for ((i=0; i < ${#LINES[@]}; i++)); do
      START=$(echo "${LINES[$i]}" | cut -d';' -f1)
      END=$(echo "${LINES[$i+1]}" | cut -d';' -f1)
      [ -z "$END" ] && END="$BAG_REAL_DURATION"

      RAW_TEXT=$(echo "${LINES[$i]}" | cut -d';' -f2-)

      # --- Fase ---
      PHASE_PART=$(echo "$RAW_TEXT" | cut -d'|' -f1)
      PHASE_PART=${PHASE_PART#"msg: '"}
      PHASE_PART=${PHASE_PART%"'"}
      PHASE_PART=$(echo "$PHASE_PART" | sed 's/^ *//; s/ *$//')

      if [[ "$PHASE_PART" == *";"* ]]; then
          # Hay subfase
          PHASE_TEXT=$(echo "$PHASE_PART" | cut -d';' -f1 | sed 's/^ *//; s/ *$//')
          SUBPHASE_TEXT=$(echo "$PHASE_PART" | cut -d';' -f2- | sed 's/^ *//; s/ *$//')

          PHASE_TEXT_ESCAPED=$(echo "$PHASE_TEXT" | sed "s/'/\\\\'/g" | sed 's/:/\\:/g')
          echo ",drawtext=text='$PHASE_TEXT_ESCAPED':fontcolor=white:fontsize=30:x=$PHASE_POSITION_X:y=$PHASE_POSITION_Y:enable='between(t,$START,$END)'" >> "$FILTER_FILE"

          SUBPHASE_TEXT_ESCAPED=$(echo "$SUBPHASE_TEXT" | sed "s/'/\\\\'/g" | sed 's/:/\\:/g')
          echo ",drawtext=text='$SUBPHASE_TEXT_ESCAPED':fontcolor=white:fontsize=24:x=$SUBPHASE_POSITION_X:y=$SUBPHASE_POSITION_Y:enable='between(t,$START,$END)'" >> "$FILTER_FILE"
      else
          # Solo fase
          PHASE_TEXT_ESCAPED=$(echo "$PHASE_PART" | sed "s/'/\\\\'/g" | sed 's/:/\\:/g')
          echo ",drawtext=text='$PHASE_TEXT_ESCAPED':fontcolor=white:fontsize=30:x=$PHASE_POSITION_X:y=$PHASE_POSITION_Y:enable='between(t,$START,$END)'" >> "$FILTER_FILE"
      fi

      # --- Variables ---
      if [[ "$RAW_TEXT" == *"|"* ]]; then
        VARS_PART=$(echo "$RAW_TEXT" | cut -d'|' -f2-)
        VARS_PART=$(echo "$VARS_PART" | sed 's/^ *//; s/ *$//')

        if [ -n "$VARS_PART" ]; then
          IFS='|' read -ra VARS_ARRAY <<< "$VARS_PART"

          for ((v=0; v<${#VARS_ARRAY[@]}; v++)); do
            var_entry=$(echo "${VARS_ARRAY[$v]}" | sed 's/^ *//; s/ *$//')

            # Eliminar comillas simples iniciales y finales de var_entry si existen
            var_entry=${var_entry#"'"}
            var_entry=${var_entry%"'"}

            if [ -n "$var_entry" ]; then
              IFS=';' read -ra parts <<< "$var_entry"
              var_name=$(echo "${parts[0]}" | sed 's/^ *//; s/ *$//')
              var_value=$(echo "${parts[1]}" | sed 's/^ *//; s/ *$//')
              var_status=$(echo "${parts[2]}" | sed 's/^ *//; s/ *$//')

              var_line="$var_name: $var_value ($var_status)"
              VAR_ESCAPED=$(echo "$var_line" | sed "s/'/\\\\'/g" | sed 's/:/\\:/g')

              echo ",drawtext=text='$VAR_ESCAPED':fontcolor=white:fontsize=18:x=$VAR_POSITION_X:y=$(($VAR_Y_OFFSET + v*$VAR_LINE_HEIGHT)):enable='between(t,$START,$END)'" >> "$FILTER_FILE"
            fi
          done
        fi
      else
        # No hay variables, no hacer nada aquí
        :
      fi
    done
  else
    echo "⚠️ No se encontró el archivo $FASES_FILE. No se añade texto."
  fi

  echo "ℹ️ Ejecutando FFmpeg con -filter_complex_script"
  ffmpeg -y \
    -i "$OUT2/bag_video_30fps.mp4" \
    -i "$OUT1/bag_video_30fps.mp4" \
    -i "$OUT3/bag_video_30fps.mp4" \
    -filter_complex_script "$FILTER_FILE" \
    -c:v libx264 -crf 18 -preset veryfast "$OUTPUT_VIDEO"

  echo "✅ Vídeo final guardado en $OUTPUT_VIDEO"
}





# ========== EJECUCIÓN ==========

if [ -n "$BAG_FILE" ] && [ -d "$BAG_FILE" ]; then
  extract_from_bag
  create_video_from_bag_frames "$OUT1"
  create_video_from_bag_frames "$OUT2"
  create_video_from_bag_frames "$OUT3"
else
  echo "⚠️ Bag file no especificado o no encontrado: $BAG_FILE"
fi



if [ -f "$OUT1/bag_video_30fps.mp4" ] && [ -f "$OUT2/bag_video_30fps.mp4" ] && [ -f "$OUT3/bag_video_30fps.mp4" ]; then
  combine_videos_side_by_side
else
  echo "⚠️ No se pueden combinar videos: falta alguno."
fi


rm -rf "$OUT1"
rm -rf "$OUT2"
rm -rf "$OUT3"

rm -rf "$FASES_FILE"
rm -rf "$FILTER_FILE"

pkill -f awk
pkill -f image_saver
pkill -f extract_frames_
  

# kill -TERM -$BAG_PGID -$PGID1 -$PGID2 -$PGID3 -$PGID_FASES