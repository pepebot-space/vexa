.PHONY: install init-db run

install:
	@PYTHON312=$$(command -v python3.12 || true); \
	if [ -z "$$PYTHON312" ]; then \
		echo "python3.12 not found in PATH. Please install Python 3.12."; \
		exit 1; \
	fi; \
	poetry env use "$$PYTHON312"
	poetry install --no-root

init:
	poetry run python database.py
	@echo "SQLite Database initialized!"

run:
	poetry run python robot.py
