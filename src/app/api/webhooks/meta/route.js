import { NextResponse } from 'next/server';
import { processMessage } from '../../../../logic/flow';
export async function GET(request) {
  const { searchParams } = new URL(request.url);
  const verifyToken = process.env.META_VERIFY_TOKEN;

  if (searchParams.get('hub.mode') === 'subscribe' && searchParams.get('hub.verify_token') === verifyToken) {
    return new NextResponse(searchParams.get('hub.challenge'), { status: 200 });
  }
  return new NextResponse('Forbidden', { status: 403 });
}

export async function POST(request) {
  try {
    const body = await request.json();
    
    if (body.object === 'page') {
      body.entry.forEach((entry) => {
        const event = entry.messaging ? entry.messaging[0] : null;
        if (event && event.message && event.message.text) {
          processMessage(event.sender.id, event.message.text).catch(console.error);
        }
      });
      return new NextResponse('EVENT_RECEIVED', { status: 200 });
    }
    return new NextResponse('Not Found', { status: 404 });
  } catch (error) {
    return new NextResponse('Error', { status: 500 });
  }
}
