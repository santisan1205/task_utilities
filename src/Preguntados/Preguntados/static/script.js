
var preguntas = {
  "Conocimiento General": [
    { texto: "Pregunta 1", opciones: ["A", "B", "C", "D"], correcta: 1 },
    { texto: "Pregunta 2", opciones: ["A", "B", "C", "D"], correcta: 1 },
    { texto: "Pregunta 3", opciones: ["A", "B", "C", "D"], correcta: 1 }
  ],
  "Historia y Mitología": [
    { texto: "Pregunta 1", opciones: ["A", "B", "C", "D"], correcta: 1 }
  ],
  "Ciencia y Tecnología": [
    { texto: "Pregunta 1", opciones: ["A", "B", "C", "D"], correcta: 1 }
  ],
  "Geografía": [
    { texto: "Pregunta 1", opciones: ["A", "B", "C", "D"], correcta: 1 }
  ],
  "Entretenimiento": [
    { texto: "Pregunta 1", opciones: ["A", "B", "C", "D"], correcta: 1 }
  ]
};

var puntaje = 0;
var categoriaActual = "";

var inicio = document.getElementById("inicio");
var pantallaPregunta = document.getElementById("pregunta");
var pantallaResultado = document.getElementById("resultado");

var categoriaNombre = document.getElementById("categoriaNombre");
var textoPregunta = document.getElementById("textoPregunta");
var opciones = document.getElementById("opciones");
var iconoResultado = document.getElementById("iconoResultado");
var textoResultado = document.getElementById("textoResultado");
var puntajeTexto = document.getElementById("puntaje");
var volver = document.getElementById("volver");

// EVENTO BOTÓN REINICIAR

document.getElementById("reiniciar").addEventListener("click", function () {
  puntaje = 0;
  document.getElementById("puntaje").textContent = puntaje;
  document.getElementById("puntaje-actual").textContent = puntaje;

  var cats = document.querySelectorAll(".categoria");
  for (var i = 0; i < cats.length; i++) {
      cats[i].classList.remove("destacada");
  }
});

// CLICK EN CATEGORÍA
var catBtns = document.querySelectorAll(".categoria");
for (var i = 0; i < catBtns.length; i++) {
  catBtns[i].addEventListener("click", function () {
      mostrarPregunta(this.getAttribute("data-categoria"));
  });
}

// MOSTRAR UNA PREGUNTA
function mostrarPregunta(categoria) {
  categoriaActual = categoria;
  var lista = preguntas[categoria];
  var pregunta = lista[Math.floor(Math.random() * lista.length)];

  inicio.className = "pantalla";
  pantallaPregunta.className = "pantalla visible";

  categoriaNombre.textContent = categoria;
  textoPregunta.textContent = pregunta.texto;
  opciones.innerHTML = "";

  for (var i = 0; i < pregunta.opciones.length; i++) {
      var boton = document.createElement("button");
      boton.textContent = String.fromCharCode(65 + i) + ". " + pregunta.opciones[i];

      // CERRAR SOBRE EL VALOR DE i
      boton.setAttribute("data-index", i);

      boton.onclick = function () {
          var idx = parseInt(this.getAttribute("data-index"));
          verificarRespuesta(idx === pregunta.correcta);
      };

      opciones.appendChild(boton);
  }
}

// VERIFICAR RESPUESTA
function verificarRespuesta(correcta) {
  pantallaPregunta.className = "pantalla";
  pantallaResultado.className = "pantalla visible";

  if (correcta) {
      textoResultado.textContent = "CORRECTO +100 PUNTOS";
      iconoResultado.innerHTML = '<img src="correcto.png" class="icono-resultado-img">';
      puntaje += 100;
  } else {
      textoResultado.textContent = "INCORRECTO";
      iconoResultado.innerHTML = '<img src="incorrecto.png" class="icono-resultado-img">';
  }

  puntajeTexto.textContent = puntaje;
  document.getElementById("puntaje-actual").textContent = puntaje;
}

// VOLVER AL INICIO
volver.addEventListener("click", function () {
  pantallaResultado.className = "pantalla";
  inicio.className = "pantalla visible";
});

// RULETA (GIRO DE CATEGORÍAS)
var botonGirar = document.getElementById("girar");
var categorias = document.querySelectorAll(".categoria");

var intervalo;
var indiceActual = 0;
var girando = false;

botonGirar.addEventListener("click", function () {
  if (girando) return;
  girando = true;

  var vueltas = 15 + Math.floor(Math.random() * 10);
  var contador = 0;

  intervalo = setInterval(function () {

      // quitar destacada
      for (var i = 0; i < categorias.length; i++) {
          categorias[i].classList.remove("destacada");
      }

      // agregar destacada
      categorias[indiceActual].classList.add("destacada");

      indiceActual = (indiceActual + 1) % categorias.length;
      contador++;

      if (contador >= vueltas) {
          clearInterval(intervalo);
          girando = false;

          var idxSel = (indiceActual - 1 + categorias.length) % categorias.length;
          var catElegida = categorias[idxSel].getAttribute("data-categoria");

          setTimeout(function () {
              mostrarPregunta(catElegida);
          }, 500);
      }

  }, 150);

});
