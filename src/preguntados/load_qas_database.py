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
        print(f" Descargando categoría {cat}...")
        url = f"https://opentdb.com/api.php?amount=20&category={cat}&type=multiple&difficulty=easy"
        resp = requests.get(url)
        if resp.status_code == 200:
            data = resp.json()
            if "results" in data:
                traductor = GoogleTranslator(source='auto', target='es')
                for item in data["results"]:
                    texto = f"{item['category']}|||{item['question']}|||{item['correct_answer']}|||{'|||'.join(item['incorrect_answers'])}"
                    try:
                        traducido = traductor.translate(texto)
                        partes = traducido.split("|||")
                        if len(partes) < 5:
                            print(" Error de traduccion, saltando a la siguiente pregunta...")
                            continue
                        categoria = partes[0]
                        pregunta = partes[1]
                        correcta = partes[2]
                        incorrectas = partes[3:]
                        opciones = incorrectas + [correcta]
                        random.shuffle(opciones)
                        todas.append({
                            "categoria": categoria,
                            "pregunta": pregunta,
                            "opciones": opciones,
                            "correcta": correcta
                        })
                        time.sleep(0.4)
                    except Exception as e:
                        print(f" Error de traduccion: {e}, saltando a la siguiente pregunta...")
    # Guarda el archivo completo
    with open("preguntas_completas.json", "w", encoding="utf-8") as f:
        json.dump(todas, f, ensure_ascii=False, indent=4)
    print(f" Se guardaron {len(todas)} preguntas en 'preguntas_completas.json'")

# --- Ejecutar ---
generar_base_preguntas()

