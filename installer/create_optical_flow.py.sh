#!/bin/bash
set -e
echo "=== Создание optical_flow.py ==="


if [ ! -f ~/optical_flow.py ]; then
    cat > ~/optical_flow.py << 'EOF'
#вставьте код сюда
EOF
    echo "optical_flow.py создан."
else
    echo "optical_flow.py уже существует."
fi
