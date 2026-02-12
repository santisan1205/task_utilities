const CATEGORIES = {
  frutas: ['🍎','🍌','🍊','🍇','🍐','🍍','🍓','🍉'],
  aula: ['✏️','🖊️','📘','📚','📁','📏','✂️','🖍️'],
  animales: ['🐱','🐶','🦁','🐰','🐵','🦊','🐸','🐷']
};

const THEMES = [
  {
    start: '#00c6ff',
    end: '#0072ff',
    shadow: 'rgba(0,114,255,0.25)',
    text: '#7fdcff',
    border: 'rgba(0,114,255,0.25)',
    glow: 'rgba(0,198,255,0.07)'
  },
  {
    start: '#ff4d4d',
    end: '#a80000',
    shadow: 'rgba(168,0,0,0.3)',
    text: '#ff9b9b',
    border: 'rgba(168,0,0,0.3)',
    glow: 'rgba(255,60,60,0.08)'
  }
];

const TIE_THEME = {
  start: '#9b5fff',
  end: '#5b2d91',
  shadow: 'rgba(155,95,255,0.3)',
  text: '#c7a4ff',
  border: 'rgba(155,95,255,0.3)',
  glow: 'rgba(155,95,255,0.08)'
};

let deck = [];
let firstCard = null;
let secondCard = null;
let lockBoard = false;

let totalPairsFound = 0;
let moves = 0;
let seconds = 0;
let timer = null;
let gameStarted = false;

let gameMode = null;
let currentPlayer = 0;
let players = [];

let chosenMode = null;
let chosenCategory = null;
let selectedSymbols = [];

// Elementos del DOM
const modeSoloBtn = document.getElementById('mode-solo-btn');
const modeVersusBtn = document.getElementById('mode-versus-btn');
const catFrutasBtn = document.getElementById('cat-frutas-btn');
const catAulaBtn = document.getElementById('cat-aula-btn');
const catAnimalesBtn = document.getElementById('cat-animales-btn');
const confirmSetupBtn = document.getElementById('confirm-setup');

const gameBoard = document.getElementById('game-board');
const movesDisplay = document.getElementById('moves');
const pairsDisplay = document.getElementById('pairs');
const timerDisplay = document.getElementById('timer');
const messageDisplay = document.getElementById('message');

const turnInfoDisplay = document.getElementById('turn-info');
const p1ScoreDisplay = document.getElementById('p1-score');
const p2ScoreDisplay = document.getElementById('p2-score');
const turnPlayerDisplay = document.getElementById('turn-player');
const scoreP2Wrapper = document.getElementById('score-p2');

const startBtn = document.getElementById('start-btn');
const resetBtn = document.getElementById('reset-btn');
const hintBtn = document.getElementById('hint-btn');

// Funciones del juego
function shuffle(array) {
  for (let i = array.length - 1; i > 0; i--) {
    const j = Math.floor(Math.random() * (i + 1));
    [array[i], array[j]] = [array[j], array[i]];
  }
  return array;
}

function formatTime(totalSeconds) {
  const m = Math.floor(totalSeconds / 60).toString().padStart(2, '0');
  const s = (totalSeconds % 60).toString().padStart(2, '0');
  return `${m}:${s}`;
}

function startTimer() {
  if (timer) clearInterval(timer);
  timer = setInterval(() => {
    seconds++;
    timerDisplay.textContent = formatTime(seconds);
  }, 1000);
}

function stopTimer() {
  if (timer) {
    clearInterval(timer);
    timer = null;
  }
}

function buildDeck() {
  deck = shuffle([...selectedSymbols, ...selectedSymbols]);
}

function renderBoard() {
  gameBoard.innerHTML = '';
  deck.forEach((symbol, idx) => {
    const cardEl = createCardElement(symbol, idx);
    gameBoard.appendChild(cardEl);
  });
}

function createCardElement(symbol, idx) {
  const card = document.createElement('div');
  card.className = 'card';
  card.dataset.symbol = symbol;
  card.dataset.index = idx;

  const front = document.createElement('div');
  front.className = 'card-front';
  const emojiSpan = document.createElement('span');
  emojiSpan.className = 'emoji-card';
  emojiSpan.textContent = symbol;
  front.appendChild(emojiSpan);

  const back = document.createElement('div');
  back.className = 'card-back';

  card.appendChild(front);
  card.appendChild(back);
  card.addEventListener('click', onCardClick);
  return card;
}

function onCardClick(e) {
  const card = e.currentTarget;
  if (!gameStarted) return;
  if (lockBoard) return;
  if (card.classList.contains('flipped')) return;
  if (card.classList.contains('matched')) return;
  if (card === firstCard) return;
  card.classList.add('flipped');
  if (!firstCard) {
    firstCard = card;
    return;
  }
  secondCard = card;
  moves++;
  movesDisplay.textContent = moves;
  lockBoard = true;
  checkForMatch();
}

function checkForMatch() {
  const isMatch = firstCard.dataset.symbol === secondCard.dataset.symbol;
  if (isMatch) {
    handleMatch();
  } else {
    unflipCards();
  }
}

function handleMatch() {
  firstCard.classList.add('matched');
  fetch('/api/match', { method: 'POST' });
  secondCard.classList.add('matched');
  firstCard.removeEventListener('click', onCardClick);
  secondCard.removeEventListener('click', onCardClick);
  totalPairsFound++;
  players[currentPlayer].score++;
  pairsDisplay.textContent = `${totalPairsFound}/${selectedSymbols.length}`;
  resetTurnVars();
  updateScoreAndTurnInfo();
  updateHintButton();
  if (totalPairsFound === selectedSymbols.length) {
    endGame();
  }
}

function unflipCards() {
  setTimeout(() => {
    firstCard.classList.remove('flipped');
    secondCard.classList.remove('flipped');
    resetTurnVars();
    switchPlayer();
  }, 1000);
}

function resetTurnVars() {
  firstCard = null;
  secondCard = null;
  lockBoard = false;
}

function switchPlayer() {
  if (players.length > 1) {
    currentPlayer = (currentPlayer + 1) % players.length;
    applyPlayerTheme(currentPlayer);
    updateScoreAndTurnInfo();
    updateHintButton();
  }
}

function showHint() {
  if (!gameStarted) return;
  if (players[currentPlayer].hintUsed) return;
  players[currentPlayer].hintUsed = true;
  updateHintButton();
  const hiddenCards = document.querySelectorAll('.card:not(.flipped):not(.matched)');
  hiddenCards.forEach(card => { card.classList.add('flipped'); });
  setTimeout(() => {
    hiddenCards.forEach(card => { card.classList.remove('flipped'); });
  }, 1000);
}

function applyPlayerTheme(playerIdx) {
  const theme = THEMES[playerIdx] || THEMES[0];
  const root = document.documentElement.style;
  root.setProperty('--accent-start', theme.start);
  root.setProperty('--accent-end', theme.end);
  root.setProperty('--accent-shadow', theme.shadow);
  root.setProperty('--accent-text', theme.text);
  root.setProperty('--accent-border', theme.border);
  root.setProperty('--accent-glow', theme.glow);
}

function applyTieTheme() {
  const theme = TIE_THEME;
  const root = document.documentElement.style;
  root.setProperty('--accent-start', theme.start);
  root.setProperty('--accent-end', theme.end);
  root.setProperty('--accent-shadow', theme.shadow);
  root.setProperty('--accent-text', theme.text);
  root.setProperty('--accent-border', theme.border);
  root.setProperty('--accent-glow', theme.glow);
}

function updateScoreAndTurnInfo() {
  p1ScoreDisplay.textContent = players[0] ? players[0].score : 0;
  p2ScoreDisplay.textContent = players[1] ? players[1].score : 0;
  if (gameMode === 'solo') {
    turnInfoDisplay.style.display = 'none';
  } else if (gameMode === 'versus') {
    turnInfoDisplay.style.display = 'flex';
    scoreP2Wrapper.style.display = 'block';
    turnPlayerDisplay.textContent = currentPlayer === 0 ? 'J1' : 'J2';
  } else {
    turnInfoDisplay.style.display = 'none';
  }
}

function updateHintButton() {
  if (!gameStarted) {
    hintBtn.disabled = true;
    hintBtn.textContent = 'Pista';
    return;
  }
  if (gameMode === 'solo') {
    if (players[0].hintUsed) {
      hintBtn.disabled = true;
      hintBtn.textContent = 'Pista usada';
    } else {
      hintBtn.disabled = false;
      hintBtn.textContent = 'Pista';
    }
  } else if (gameMode === 'versus') {
    const p = players[currentPlayer];
    if (p.hintUsed) {
      hintBtn.disabled = true;
      hintBtn.textContent = `Pista J${currentPlayer + 1} usada`;
    } else {
      hintBtn.disabled = false;
      hintBtn.textContent = `Pista J${currentPlayer + 1}`;
    }
  } else {
    hintBtn.disabled = true;
    hintBtn.textContent = 'Pista';
  }
}

function clearActiveOptions() {
  [modeSoloBtn, modeVersusBtn, catFrutasBtn, catAulaBtn, catAnimalesBtn].forEach(btn => {
    btn.classList.remove('active');
  });
}

function selectMode(mode) {
  chosenMode = mode;
  modeSoloBtn.classList.toggle('active', mode === 'solo');
  modeVersusBtn.classList.toggle('active', mode === 'versus');
  maybeEnableConfirm();
}

function selectCategory(cat) {
  chosenCategory = cat;
  catFrutasBtn.classList.toggle('active', cat === 'frutas');
  catAulaBtn.classList.toggle('active', cat === 'aula');
  catAnimalesBtn.classList.toggle('active', cat === 'animales');
  maybeEnableConfirm();
}

function maybeEnableConfirm() {
  confirmSetupBtn.disabled = !(chosenMode && chosenCategory);
}

function finalizeSetupAndInitGame() {
  gameMode = chosenMode;
  selectedSymbols = CATEGORIES[chosenCategory];
  if (gameMode === 'solo') {
    players = [{ score: 0, hintUsed: false }];
  } else {
    players = [{ score: 0, hintUsed: false }, { score: 0, hintUsed: false }];
  }
  currentPlayer = 0;
  applyPlayerTheme(currentPlayer);
  totalPairsFound = 0;
  moves = 0;
  seconds = 0;
  gameStarted = false;
  stopTimer();
  buildDeck();
  renderBoard();
  movesDisplay.textContent = moves;
  pairsDisplay.textContent = `${totalPairsFound}/${selectedSymbols.length}`;
  timerDisplay.textContent = '00:00';
  messageDisplay.textContent = 'Pulsa "Iniciar juego" para comenzar.';
  messageDisplay.classList.remove('win-message');
  updateScoreAndTurnInfo();
  
  // CORRECCIÓN: Habilitar el botón de inicio correctamente
  startBtn.disabled = false;
  resetBtn.disabled = true;
  hintBtn.disabled = true;
  updateHintButton();
}

function startGame() {
  if (gameStarted) return;
  fetch('/api/start', { method: 'POST' });
  gameStarted = true;
  seconds = 0;
  timerDisplay.textContent = '00:00';
  stopTimer();
  startTimer();
  messageDisplay.textContent = '¡Encuentra todas las parejas!';
  startBtn.disabled = true;
  resetBtn.disabled = false;
  updateHintButton();
}

function resetGame() {
  stopTimer();
  fetch('/api/reset', { method: 'POST' });
  gameStarted = false;
  deck = [];
  players = [];
  gameMode = null;
  currentPlayer = 0;
  totalPairsFound = 0;
  moves = 0;
  seconds = 0;
  selectedSymbols = [];
  chosenMode = null;
  chosenCategory = null;
  applyPlayerTheme(0);
  gameBoard.innerHTML = '';
  movesDisplay.textContent = '0';
  pairsDisplay.textContent = '0/8';
  timerDisplay.textContent = '00:00';
  messageDisplay.textContent = 'Selecciona modo y tema para empezar.';
  messageDisplay.classList.remove('win-message');
  turnInfoDisplay.style.display = 'none';
  p1ScoreDisplay.textContent = '0';
  p2ScoreDisplay.textContent = '0';
  turnPlayerDisplay.textContent = '';
  
  // CORRECCIÓN: Deshabilitar botones correctamente
  startBtn.disabled = true;
  resetBtn.disabled = true;
  hintBtn.disabled = true;
  hintBtn.textContent = 'Pista';
  clearActiveOptions();
  confirmSetupBtn.disabled = true;
}

function endGame() {
  stopTimer();
  gameStarted = false;
  if (gameMode === 'solo') {
    applyPlayerTheme(0);
    messageDisplay.textContent = `¡Felicidades! Completaste el juego en ${moves} movimientos y ${timerDisplay.textContent}.`;
    messageDisplay.classList.add('win-message');
    hintBtn.disabled = true;
    startBtn.disabled = false;
    resetBtn.disabled = false;
  } else {
    const p1 = players[0].score;
    const p2 = players[1].score;
    let winnerMsg = '';
    if (p1 > p2) {
      winnerMsg = '¡Ganó Jugador 1!';
      applyPlayerTheme(0);
    } else if (p2 > p1) {
      winnerMsg = '¡Ganó Jugador 2!';
      applyPlayerTheme(1);
    } else {
      winnerMsg = '¡Empate!';
      applyTieTheme();
    }
    fetch('/api/win', { 
      method: 'POST', 
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({ winner: winnerMsg }) 
    });
    messageDisplay.textContent = `${winnerMsg} P1: ${p1} | P2: ${p2}`;
    messageDisplay.classList.add('win-message');
    hintBtn.disabled = true;
    startBtn.disabled = false;
    resetBtn.disabled = false;
  }
}

// Event listeners
modeSoloBtn.addEventListener('click', () => selectMode('solo'));
modeVersusBtn.addEventListener('click', () => selectMode('versus'));
catFrutasBtn.addEventListener('click', () => selectCategory('frutas'));
catAulaBtn.addEventListener('click', () => selectCategory('aula'));
catAnimalesBtn.addEventListener('click', () => selectCategory('animales'));
confirmSetupBtn.addEventListener('click', finalizeSetupAndInitGame);
startBtn.addEventListener('click', startGame);
resetBtn.addEventListener('click', resetGame);
hintBtn.addEventListener('click', showHint);

// Inicialización
document.addEventListener('DOMContentLoaded', () => {
  // Crear partículas de fondo
  createParticles();
  
  startBtn.disabled = true;
  resetBtn.disabled = true;
  hintBtn.disabled = true;
  messageDisplay.textContent = 'Selecciona modo y tema para empezar.';
});

// Función para crear partículas de fondo
function createParticles() {
  const particlesContainer = document.getElementById('particles');
  const particleCount = 30;
  
  for (let i = 0; i < particleCount; i++) {
    const particle = document.createElement('div');
    particle.className = 'particle';
    
    // Tamaño y posición aleatorios
    const size = Math.random() * 5 + 2;
    const posX = Math.random() * 100;
    const delay = Math.random() * 15;
    
    particle.style.width = `${size}px`;
    particle.style.height = `${size}px`;
    particle.style.left = `${posX}%`;
    particle.style.animationDelay = `${delay}s`;
    
    particlesContainer.appendChild(particle);
  }
}
