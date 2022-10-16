

function buttonEvent(idButton)
{
    console.log(idButton);
    switch(idButton)
    {
        case 'LFront':
            writeConsole('Avant Gauche<br>');
            updateTwist(1,1);
            break;
        case 'Front':
            writeConsole('Avant<br>');
            updateTwist(1,0);
            break;
        case 'RFront':
            writeConsole('Avant Droite<br>');
            updateTwist(1,-1);
            break;
        case 'Left':
            writeConsole('Gauche<br>');
            updateTwist(0,1);
            break;
        case 'Stop':
            writeConsole('Stop<br>');
            updateTwist(0,0);
            break;
        case 'Right':
            writeConsole('Droite<br>');
            updateTwist(0,-1);
            break;
        case 'LBack':
            writeConsole('Arrière Gauche<br>');
            updateTwist(-1,1);
            break;
        case 'Back':
            writeConsole('Arrière<br>');
            updateTwist(-1,0);
            break;
        case 'RBack':
            writeConsole('Arrière Droite<br>');
            updateTwist(-1,-1);
            break;

    }
}

function writeConsole(text)
{
  // document.getElementById()
  let console = document.getElementById('consoleLog');
  console.innerHTML += text;
  console.scrollTop = console.scrollHeight;
}

function clearConsole()
{
  document.getElementById('consoleLog').innerHTML = '';
}   

function updateTwist(linear,angular)
{
    twist.linear.x = linear;
    twist.angular.z = angular;
}

function disableControlButtons(state)
{
    let buttons = document.getElementById('buttonsControl').getElementsByTagName('button');
    // console.log(buttons);

    if(state == 1)
    {
        for(let i=0; i< buttons.length; i++)
        {
            buttons[i].disabled = true;
            buttons[i].className = 'greyed-Button';
            // console.log(buttons[i].id)
        }
    }
    else
    {
        for(let i=0; i< buttons.length; i++)
        {
            buttons[i].disabled = false;
            buttons[i].className = 'button-style';
        }
    }
}