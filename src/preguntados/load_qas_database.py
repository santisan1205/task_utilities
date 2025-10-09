import requests
import json
import random
from deep_translator import GoogleTranslator
import time

def generar_base_preguntas():
    """Genera una base de preguntas completa con varias categorías"""
    todas = []
    categorias = [
        "9", "10", "11", "14", "15", "17", "18", "19", "20", "21", "22", "23"]
    for cat in categorias:
        # Descarga las preguntas de la API en formato JSON que posteriormente se trabaja como dict
        print(f" Descargando categoría {cat}...")
        url = f"https://opentdb.com/api.php?amount=20&category={cat}&type=multiple&difficulty=easy"
        resp = requests.get(url)
        if resp.status_code == 200:
            data = resp.json()
            # Traducir las preguntas y respuestas al español, si no es posible la omite.
            if "results" in data:
                traductor = GoogleTranslator(source='auto', target='es')
                for item in data["results"]:
                    # Para ahorrar recursos de la API de traduccion, se traduce todo en un solo llamado
                    texto = f"{item['category']}|||{item['question']}|||{item['correct_answer']}|||{'|||'.join(item['incorrect_answers'])}"
                    try:
                        # Separa las partes traducidas
                        traducido = traductor.translate(texto)
                        partes = traducido.split("|||")
                        # Verifica que se hayan traducido todas las partes, si no es así, omite la pregunta
                        if len(partes) < 5:
                            print(" Error de traduccion, saltando a la siguiente pregunta...")
                            continue
                        categoria = partes[0]
                        pregunta = partes[1]
                        correcta = partes[2]
                        incorrectas = partes[3:]
                        opciones = incorrectas + [correcta]
                        random.shuffle(opciones)
                        # Guarda la pregunta en la lista completa
                        todas.append({
                            "categoria": categoria,
                            "pregunta": pregunta,
                            "opciones": opciones,
                            "correcta": correcta
                        })
                        # Pausa para evitar saturar la API
                        time.sleep(0.4)
                    # Manejo de errores en la traduccion
                    except Exception as e:
                        print(f" Error de traduccion: {e}, saltando a la siguiente pregunta...")
    # Guarda el archivo completo con la base de datos
    with open("preguntas_database.json", "w", encoding="utf-8") as f:
        json.dump(todas, f, ensure_ascii=False, indent=4)
    print(f" Se guardaron {len(todas)} preguntas en 'preguntas_database.json'")

# --- Ejecutar ---
generar_base_preguntas()

