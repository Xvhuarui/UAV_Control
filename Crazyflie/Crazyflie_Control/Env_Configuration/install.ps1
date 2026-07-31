#wget -O ~\Downloads\Python_setup.exe https://www.python.org/ftp/python/3.12.10/python-3.12.10-amd64.exe
#~\Downloads\Python_setup.exe /quiet TargetDir=C:\Python\Python312 InstallAllUsers=1 PrependPath=1 Include_test=0

pip config set global.index-url https://mirrors.tuna.tsinghua.edu.cn/pypi/web/simple
pip config set global.trusted-host mirrors.tuna.tsinghua.edu.cn
pip install poetry poetry-plugin-shell
#poetry config virtualenvs.create false --local
poetry config virtualenvs.in-project true
poetry lock
poetry install
