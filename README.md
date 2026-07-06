# Shortest_Path_Project
A highly optimized implementation of Dijkstra's Shortest Path Algorithm in python capable of parsing real-world geospatial data (.graphml maps) to find the most efficient routes.

# Shortest Path Algorithm for Real-World Maps (Python)

This project implements Dijkstra's Shortest Path Algorithm, A* Algorithm and Double A* Algorithm in Python to calculate the absolute shortest routes over large-scale road networks. 

## 🗺️ Project Focus
The core engine is built to process real-world spatial networks (originally tested using an OpenStreetMap graph layout of Bengaluru, India). 

## 🚀 Key Specifications
- **Data Structure:** Uses Python's built-in `heapq` module (binary heap) to drastically optimize the priority queue operations.
- **Time Complexity:** $O((V + E) \log V)$, making it efficient enough to calculate routes over thousands of intersections in seconds.
- **Input Format:** Parses `.graphml` graph files into nodes (intersections) and weighted edges (road distances/travel times).

## 💻 Tech Stack
- **Language:** Python 3
- **Key Modules:** `heapq` (Min-Heap), `os`, `osmnx`(open street maps), `flask`
- **Concepts:** Dijkstra's Algorithm, A* Algorithm, Double A* Algorithm, Graph Theory, Network Routing

---
## 🛠️ Setup & How to Run

Follow these steps to run the project locally.

### 1. Clone the Repository

```bash
git clone <repository-url>
cd <repository-folder>
```

Replace `<repository-url>` with your GitHub repository URL and `<repository-folder>` with the cloned folder name.

### 2. Create a Virtual Environment

**Windows**

```bash
python -m venv venv
venv\Scripts\activate
```

**macOS / Linux**

```bash
python3 -m venv venv
source venv/bin/activate
```

### 3. Install Dependencies

```bash
pip install -r requirements.txt
```

### 4. Run the Application

```bash
python app.py
```

Once the Flask server starts, open your browser and navigate to the local URL displayed in the terminal (typically `http://127.0.0.1:5000`).
