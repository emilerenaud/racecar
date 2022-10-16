

function buttonEvent(text,state)
{
  writeConsole(text);
  updateTwist(state);
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

function updateTwist(state)
{
    if(state == 1 || state ==0)
    {
        twist.linear.x = state;
    }
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