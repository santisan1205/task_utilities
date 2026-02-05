#!/usr/bin/env python3
import rospy
from fastapi import FastAPI, Request
from fastapi.staticfiles import StaticFiles
from fastapi.responses import RedirectResponse, HTMLResponse
from robot_toolkit_msgs.msg import speech_recognition_status_msg
import os

rospy.init_node("tablet_node")
hot_word_publisher = rospy.Publisher("/pytoolkit/ALSpeechRecognition/status", speech_recognition_status_msg, queue_size=10)

app = FastAPI()

app.mount("/static", StaticFiles(directory="./src/task_utilities/src/static"), name="static")

@app.get("/", response_class=RedirectResponse)
async def main_page():
    return RedirectResponse(url="/static/boton.html")

@app.post("/jugar", response_class=HTMLResponse)
@app.head("/jugar", response_class=HTMLResponse)
async def handle_jugar(request: Request):
    form_data = await request.form()
    action = form_data.get('action')
    print(action)
    hot_word_publisher.publish(action)
    file_path = os.path.join("./src/task_utilities/src/static", "boton.html")
    
    # Read the file content
    with open(file_path, 'r') as file:
        html_content = file.read()

    return HTMLResponse(content="")

@app.post("/action", response_class=HTMLResponse)
@app.head("/action", response_class=HTMLResponse)
async def handle_action(request: Request):
    form_data = await request.form()
    action = form_data.get('action')
    print(action)
    hot_word_publisher.publish(action)
    file_path = os.path.join("./src/task_utilities/src/static", "preguntas.html")
    
    # Read the file content
    with open(file_path, 'r') as file:
        html_content = file.read()

    return HTMLResponse(content=html_content)



if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)
