# Research Guide

Panduan untuk membuat research baru di project ini.

## Struktur Folder Research

Setiap research baru akan memiliki folder sendiri di dalam direktori `research/`:

```
research/
├── <nama_research>_<arxiv_id>/    # jika ada paper arxiv
├── <nama_research>/               # jika tidak ada paper reference
│   ├── README.md
│   ├── pyproject.toml
│   ├── Makefile
│   └── src/
│       └── ...
```

## Langkah Membuat Research Baru

### 1. Buat Folder Research

```bash
# Dengan arxiv id
mkdir -p research/my_research_2301.12345

# Tanpa arxiv id
mkdir -p research/my_research
```

### 2. Buat File README.md

Template README.md:

```markdown
# Nama Research

## Paper Reference
- Judul: [Judul Paper]
- Link: [URL Paper]
- Arxiv ID: [ID jika ada]

## Deskripsi
[Penjelasan singkat tentang research dan tujuannya]

## Method / Formula

### Formula Utama
[Penjelasan formula yang digunakan dengan notasi matematika]

Contoh:
```
y = f(x) = Wx + b

dimana:
- W = weight matrix
- b = bias vector
- x = input
```

### Algoritma
[Langkah-langkah algoritma yang digunakan]

### Implementasi
[Detail implementasi dan pertimbangan teknis]

## Dependencies

Lihat file `pyproject.toml` untuk daftar lengkap dependencies.

### Install Dependencies

```bash
make install
```

## Cara Menjalankan

### Setup Environment

```bash
make setup
```

### Run Experiments

```bash
make run
```

### Run Tests

```bash
make test
```

### Lint dan Format

```bash
make lint
make format
```

## Struktur Kode

```
src/
├── __init__.py
├── main.py          # Entry point
├── model.py         # Implementasi model
├── data.py          # Data loading dan preprocessing
└── utils.py         # Utility functions
```

## Hasil dan Eksperimen

### Eksperimen 1
- Deskripsi: [...]
- Hasil: [...]
- Catatan: [...]

## Referensi Tambahan

1. [Referensi 1]
2. [Referensi 2]
```

### 3. Buat File pyproject.toml

Template pyproject.toml:

```toml
[project]
name = "nama_research"
version = "0.1.0"
description = "Deskripsi singkat research"
requires-python = ">=3.10"
dependencies = [
    "numpy>=1.24.0",
    "torch>=2.0.0",
    # Tambahkan dependencies lain
]

[project.optional-dependencies]
dev = [
    "pytest>=7.0.0",
    "ruff>=0.1.0",
    "mypy>=1.0.0",
]

[build-system]
requires = ["hatchling"]
build-backend = "hatchling.build"

[tool.ruff]
line-length = 120
target-version = "py310"

[tool.mypy]
python_version = "3.10"
strict = true
```

### 4. Buat File Makefile

Template Makefile:

```makefile
.PHONY: install setup run test lint format clean

PYTHON := python3
VENV := .venv
BIN := $(VENV)/bin

install:
	pip install -e ".[dev]"

setup: install
	$(BIN)/python -c "import torch; print('Setup complete')"

run: setup
	$(BIN)/python src/main.py

test: setup
	$(BIN)/pytest tests/ -v

lint:
	$(BIN)/ruff check src/

format:
	$(BIN)/ruff format src/

clean:
	rm -rf $(VENV)
	rm -rf __pycache__
	rm -rf .pytest_cache
	find . -type d -name "__pycache__" -exec rm -rf {} +
```

## Checklist Research Baru

- [ ] Folder research dibuat dengan nama yang sesuai
- [ ] README.md dengan deskripsi lengkap
- [ ] Method/formula dijelaskan dengan jelas
- [ ] pyproject.toml dengan dependencies yang diperlukan
- [ ] Makefile dengan command untuk setup dan run
- [ ] Struktur kode src/ dibuat
- [ ] Tests ditulis (opsional tapi direkomendasikan)
- [ ] Dokumentasi hasil eksperimen di README.md

## Konvensi Penamaan

| Tipe | Format | Contoh |
|------|--------|--------|
| Dengan arxiv | `<nama>_<arxiv_id>` | `attention_transformer_1706.03762` |
| Tanpa arxiv | `<nama>` | `sentiment_analysis` |
| Multi-paper | `<nama>_<id1>_<id2>` | `multi_attention_1706.03762_1810.04805` |

gunakan snake_case untuk nama research.

## Tips

1. Selalu tulis formula dengan notasi yang jelas di README
2. Tulis referensi paper di bagian atas README
3. Update hasil eksperimen secara berkala
4. Gunakan virtual environment terpisah untuk setiap research
5. Commit progress secara berkala