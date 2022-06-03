switch (movimiento)
      {
        case 'A':
          moveCar();
          break;
        case 'B':
          backCar();
          break;
        case 'C':
          rightCar();
          break;
        case 'D':
          leftCar();
          break;
        case 'E':
          turnCounterClockwise();
          break;
        case 'F':
          turnClockwise();
          break;
        case 'G':
          diagonalMR();
          break;
        case 'H':
          diagonalRB();
          break;
        case 'I':
          diagonalBL();
          break;
        case 'J':
          diagonalLM();
          break;
        case 'K':
          stopCar();
          break;
        case 'L':
          stopCarF();
          break;
        default:
          stopCar();
      }