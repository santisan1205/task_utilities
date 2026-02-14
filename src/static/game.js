/* CÓDIGO JAVASCRIPT ES5 PARA PEPPER (LEGACY) */

// Variables Globales
var CATEGORIES = {
  'frutas': ['🍎','🍌','🍊','🍇','🍐','🍍','🍓','🍉'],
  'aula': ['✏️','🖊️','📘','📚','📁','📏','✂️','🖍️'],
  'animales': ['🐱','🐶','🦁','🐰','🐵','🦊','🐸','🐷']
};

var deck = [];
var firstCard = null;
var secondCard = null;
var lockBoard = false;
var totalPairsFound = 0;
var moves = 0;
var seconds = 0;
var timerInterval = null;
var gameStarted = false;
var gameMode = null;
var selectedSymbols = [];
var chosenMode = null;
var chosenCategory = null;

// Referencias DOM
var startBtn = document.getElementById('start-btn');
var resetBtn = document.getElementById('reset-btn');
var confirmSetupBtn = document.getElementById('confirm-setup');
var gameBoard = document.getElementById('game-board');
var movesDisplay = document.getElementById('moves');
var pairsDisplay = document.getElementById('pairs');
var timerDisplay = document.getElementById('timer');
var messageDisplay = document.getElementById('message');

// --- AJAX HELPER (Reemplazo de FETCH) ---
function sendSignal(endpoint, data) {
    var xhr = new XMLHttpRequest();
    xhr.open("POST", "/api/" + endpoint, true);
    xhr.setRequestHeader("Content-Type", "application/json");
    xhr.onreadystatechange = function() {
        if (xhr.readyState === 4 && xhr.status === 200) {
            console.log("Signal sent: " + endpoint);
        }
    };
    var payload = data ? JSON.stringify(data) : "{}";
    xhr.send(payload);
}

// --- CONFIGURACIÓN ---
function selectMode(btn, mode) {
    chosenMode = mode;
    // Limpiar clases active
    document.getElementById('mode-solo-btn').className = "option-btn";
    document.getElementById('mode-versus-btn').className = "option-btn";
    // Activar actual
    btn.className = "option-btn active";
    checkConfig();
}

function selectCategory(btn, cat) {
    chosenCategory = cat;
    document.getElementById('cat-frutas-btn').className = "option-btn";
    document.getElementById('cat-aula-btn').className = "option-btn";
    document.getElementById('cat-animales-btn').className = "option-btn";
    btn.className = "option-btn active";
    checkConfig();
}

function checkConfig() {
    if (chosenMode && chosenCategory) {
        confirmSetupBtn.disabled = false;
    }
}

function finalizeSetup() {
    gameMode = chosenMode;
    selectedSymbols = CATEGORIES[chosenCategory];
    resetGameLogic();
    buildDeck();
    renderBoard();
    
    startBtn.disabled = false;
    resetBtn.disabled = true;
    confirmSetupBtn.disabled = true;
    
    messageDisplay.innerText = 'Pulsa "Iniciar" para comenzar.';
}

// --- LÓGICA DE JUEGO ---

function shuffle(array) {
    var currentIndex = array.length, temporaryValue, randomIndex;
    while (0 !== currentIndex) {
        randomIndex = Math.floor(Math.random() * currentIndex);
        currentIndex -= 1;
        temporaryValue = array[currentIndex];
        array[currentIndex] = array[randomIndex];
        array[randomIndex] = temporaryValue;
    }
    return array;
}

function buildDeck() {
    // Duplicar array (ES5 style)
    var doubled = selectedSymbols.concat(selectedSymbols);
    deck = shuffle(doubled);
}

function renderBoard() {
    gameBoard.innerHTML = ''; // Limpiar
    for (var i = 0; i < deck.length; i++) {
        var symbol = deck[i];
        
        // Crear estructura HTML manualmente
        var card = document.createElement('div');
        card.className = 'card';
        card.setAttribute('data-symbol', symbol);
        card.onclick = function() { onCardClick(this); }; // Bind this

        var inner = document.createElement('div');
        inner.className = 'card-inner';

        var front = document.createElement('div'); // La parte azul con ?
        front.className = 'card-front';
        front.innerHTML = '<span class="emoji-center">?</span>';

        var back = document.createElement('div'); // La parte con emoji
        back.className = 'card-back';
        back.innerHTML = '<span class="emoji-center">' + symbol + '</span>';

        inner.appendChild(front);
        inner.appendChild(back);
        card.appendChild(inner);
        gameBoard.appendChild(card);
    }
}

function startGame() {
    if (gameStarted) return;
    gameStarted = true;
    startBtn.disabled = true;
    resetBtn.disabled = false;
    
    sendSignal("start"); // Avisar a Python
    
    startTimer();
    messageDisplay.innerText = "¡Encuentra las parejas!";
}

function startTimer() {
    if (timerInterval) clearInterval(timerInterval);
    seconds = 0;
    timerInterval = setInterval(function() {
        seconds++;
        var m = Math.floor(seconds / 60);
        var s = seconds % 60;
        // Pad simple
        var mStr = m < 10 ? "0" + m : m;
        var sStr = s < 10 ? "0" + s : s;
        timerDisplay.innerText = mStr + ":" + sStr;
    }, 1000);
}

function onCardClick(card) {
    if (!gameStarted) return;
    if (lockBoard) return;
    if (card === firstCard) return;
    // Comprobar si ya tiene clase flipped
    if (card.className.indexOf('flipped') !== -1) return; 

    // Añadir clase 'flipped'
    card.className += " flipped";

    if (!firstCard) {
        firstCard = card;
        return;
    }

    secondCard = card;
    lockBoard = true;
    moves++;
    movesDisplay.innerText = moves;

    checkForMatch();
}

function checkForMatch() {
    var s1 = firstCard.getAttribute('data-symbol');
    var s2 = secondCard.getAttribute('data-symbol');

    if (s1 === s2) {
        handleMatch();
    } else {
        unflipCards();
    }
}

function handleMatch() {
    // Añadir clase matched
    firstCard.className += " matched";
    secondCard.className += " matched";
    
    sendSignal("match"); // Avisar a Python

    totalPairsFound++;
    pairsDisplay.innerText = totalPairsFound + "/" + selectedSymbols.length;

    resetTurnVars();

    if (totalPairsFound === selectedSymbols.length) {
        endGame();
    }
}

function unflipCards() {
    setTimeout(function() {
        // Quitar clase flipped (reemplazo simple de string)
        firstCard.className = firstCard.className.replace(" flipped", "");
        secondCard.className = secondCard.className.replace(" flipped", "");
        
        // Opcional: Avisar error
        // sendSignal("mismatch"); 
        
        resetTurnVars();
    }, 1000);
}

function resetTurnVars() {
    firstCard = null;
    secondCard = null;
    lockBoard = false;
}

function resetGameLogic() {
    deck = [];
    totalPairsFound = 0;
    moves = 0;
    seconds = 0;
    if (timerInterval) clearInterval(timerInterval);
    timerDisplay.innerText = "00:00";
    movesDisplay.innerText = "0";
    pairsDisplay.innerText = "0/8";
    gameBoard.innerHTML = '';
}

function fullReset() {
    resetGameLogic();
    sendSignal("reset");
    
    gameStarted = false;
    startBtn.disabled = true;
    resetBtn.disabled = true;
    confirmSetupBtn.disabled = false;
    
    // Limpiar selección visual
    gameBoard.innerHTML = '<div style="color:white; padding:20px;">Configura nuevo juego...</div>';
    messageDisplay.innerText = "Configura y pulsa Continuar";
}

function endGame() {
    clearInterval(timerInterval);
    var msg = "¡Ganaste en " + moves + " movimientos!";
    messageDisplay.innerText = msg;
    
    sendSignal("win", { winner: "Jugador" });
}