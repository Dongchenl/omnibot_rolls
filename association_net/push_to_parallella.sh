#readonly PARALLELLA_IP=172.19.11.170 # Alphas parallella
readonly PARALLELLA_IP=10.162.177.221 # Raphaelas parallella
readonly FOLDER_NAME=omnibot_parallella_mario

# set time and date correctly
ssh root@$PARALLELLA_IP date -s "\"$(date +'%F %T')\""

# check first whether the build would succeed
make --directory ./build/

ssh root@$PARALLELLA_IP rm -r /home/parallella/code/$FOLDER_NAME/*

rsync -v -rv -e ssh --exclude='.git/*' --exclude='build' \
     --exclude='.*' \
    ./ \
    root\@$PARALLELLA_IP:/home/parallella/code/$FOLDER_NAME

ssh root@$PARALLELLA_IP mkdir /home/parallella/code/$FOLDER_NAME/build
ssh root@$PARALLELLA_IP cmake -B/home/parallella/code/$FOLDER_NAME/build -H/home/parallella/code/$FOLDER_NAME
ssh root@$PARALLELLA_IP make --directory /home/parallella/code/$FOLDER_NAME/build/
